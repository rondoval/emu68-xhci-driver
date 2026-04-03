# emu68-xhci-driver

AmigaOS xHCI USB 2.0 / USB 3.0 host controller driver for [Emu68](https://github.com/michalsc/Emu68)
on Raspberry Pi 4 (BCM2711).

The xHCI stack is derived from the [Das U-Boot](https://source.denx.de/u-boot/u-boot) xHCI driver.
The driver exposes a Poseidon-compatible HCD API and is loaded as a standard AmigaOS `.device`.
The API is not an exact match — certain fields passed by the stack are intentionally ignored
 in favour of the driver's own internal structures.

> **Note for users upgrading from pre-3.x releases:** unit numbering has changed.
> Unit 0 used to be the VL805 (PCIe).  It is now the onboard OTG port.
> Unit 1 is now the VL805.  Update your USB stack configuration accordingly.

---

## Status

The following has been verified:

- Root hub enumeration, USB 2.0 hub support, external hubs
- Control, bulk, interrupt and real-time isochronous transfers (both directions)
- HID devices, mass storage (thumb drives), USB audio cards
- Experimental SuperSpeed (USB 3.0) support, including USB 3.0 hubs

Known gaps / issues:

- Non-RT isochronous transfers not tested
- RT isochronous audio has glitches
- AHI 4.x not yet supported

> Data corruption is possible in edge cases.  Back up before heavy use.

---

## Unit numbering

| Unit | Port |
|---|---|
| 0 | Onboard OTG port (Pi 4B and CM4) |
| 1+ | PCIe xHCI controllers, indexed from 1 |

On a stock Pi 4B: unit 0 is the OTG port, unit 1 is the VL805 (four USB-A ports).
On a CM4: unit 0 is the OTG port; unit 1 exists only if a PCIe xHCI device is attached.

The OTG port requires `otg_mode=1` in `config.txt`.

---

## How the driver works

One `XHCIUnit` is created per physical xHCI controller, each managed by a dedicated task.

### Startup

On `OpenDevice()`:

1. For unit 0 (OTG), the BCM2711 onboard xHCI controller is located via the Emu68
   device tree (`/scb/xhci`).
2. For unit 1+ (PCIe), [`emu68-pcie-library`](https://github.com/rondoval/emu68-pcie-library)
   initialises the Broadcom STB PCIe controller, enumerates the bus and assigns BARs.
   If a VIA VL805 is found, its firmware is loaded via the VideoCore mailbox.
   Because `emu68-pcie-library` is statically linked, the PCIe bus state is private to
   this driver — no other driver can share the bus while it is open.
3. The xHCI controller is reset: command, event and transfer rings are allocated, the
   DCBAA is set up and the controller is started.
4. An interrupt handler is registered via `gic400.library` (MSI) or a wired IRQ line.
5. The unit task is spawned and enters its event loop.

### Transfer processing

`BeginIO()` either handles the request immediately or enqueues it to the unit's message
port.  The unit task dequeues requests and dispatches them:

- Control transfers → command ring + control TD
- Bulk / interrupt → async or periodic transfer ring
- Isochronous → periodic ring with frame-accurate scheduling

The interrupt handler signals the unit task when the event ring has entries.  The task
drains the event ring, completes pending `IORequest`s and re-arms the interrupt.  A
watchdog timer fires every 100 ms to catch missed events and enforce command-ring
timeouts (5 s).

### USB 3.0 on a USB 2.0 stack

An USB 2.0 stack has no knowledge of USB 3.0 specifics.  The driver bridges the gap
in several ways:

**Root hub emulation.** `xhci-root-hub.c` maintains both a USB 2.0 and a USB 3.0 root
hub descriptor set.  The stack always sees a USB 2.0 root hub regardless of the physical
port speed.

**SS hub emulation.** When a USB 3.0 hub is discovered it is marked with
`ss_hub_emulation`.  USB 2.0 hub control requests from the stack are then translated on
the fly to their SuperSpeed equivalents — for example `SUSPEND`/`RESUME` feature
requests are converted to `LINK_STATE` transitions, and USB 2.0-only features such as
`ENABLE`/`C_ENABLE` (which have no SS equivalent) are silently swallowed.

**SS hub port status translation.** A USB 3.0 hub reports port status using a different
set of bit positions and semantics from a USB 2.0 hub.  Before a `GET_PORT_STATUS` reply
is returned to the stack, `xhci_udev_map_ss_port_status` rewrites both the status word
and the change word in-place: the `USB_SS_PORT_STAT_POWER` bit is remapped to the USB
2.0 `USB_PORT_STAT_POWER` position; PLS=U3 is reported as `USB_PORT_STAT_SUSPEND`; the
SS speed field is translated to the USB 2.0 `LOW_SPEED`/`HIGH_SPEED` indicators (a
SuperSpeed device is presented to the stack as high-speed); and
`USB_SS_PORT_STAT_C_LINK_STATE` with PLS=U0 is mapped to `USB_PORT_STAT_C_SUSPEND`.
Other change bits (connection, over-current, reset) carry over unchanged.

**Descriptor translation.** Configuration descriptors returned to the stack have
SuperSpeed Endpoint Companion descriptors (`USB_DT_SS_ENDPOINT_COMP`) stripped out,
since the stack does not understand them.  When a hub is being configured, the driver
internally fetches the hub descriptor first so it can program the correct port count and
TT think time into the xHCI slot context before the `SET_CONFIGURATION` completes.

**Hub descriptor type selection.** The driver uses `USB_DT_SS_HUB` when talking to a
SuperSpeed hub and `USB_DT_HUB` for high-speed hubs; the stack only ever sees the
high-speed variant.

### Descriptor parsing and xHCI context creation

xHCI requires far more detail about each endpoint than a USB 2.0 stack typically
tracks.  The stack does pass descriptor data to HCD drivers — enough for USB 2.0 — but
the driver's API header intentionally ignores it.  Instead, all data used to build the
xHCI slot and endpoint contexts comes from the driver's own internal descriptor tree,
populated during enumeration, which additionally captures SuperSpeed Endpoint Companion
descriptors and other details that the stack would never see or pass through.

**Configuration descriptor ingestion.** When a device enumerates, the driver performs a
complete walk of the raw configuration descriptor bytes.  It builds an internal tree:
`usb_config` → `usb_interface[]` → `usb_interface_altsetting[]` → `usb_endpoint_descriptor[]`.
Critically, whenever a SuperSpeed Endpoint Companion descriptor
(`USB_DT_SS_ENDPOINT_COMP`) immediately follows a standard endpoint descriptor, it is
captured into a parallel `ss_ep_comp_desc[]` array alongside the endpoint — one entry per
endpoint.  This data is kept internally and is never exposed to the stack.

**Companion descriptor stripping.** Before any configuration descriptor is returned
upward, `xhci_filter_ss_ep_companion_desc` performs an in-place compaction of the raw
byte buffer: it copies every non-`USB_DT_SS_ENDPOINT_COMP` descriptor forward over the
gaps left by the stripped entries, zeroes the tail, and adjusts `wTotalLength`
accordingly.  The stack receives a well-formed, USB 2.0-shaped configuration blob.

**Slot context (`ENABLE_SLOT` / `ADDRESS_DEVICE`).** When a device receives its address,
`xhci_setup_addressable_virt_dev` programs the xHCI input slot context:

- *Route string* — `build_route_string` packs the port number at each hub tier as a
  4-bit nibble into the 20-bit route string field (bits [19:0]).  The first tier below
  the root occupies bits [3:0], the second tier [7:4], and so on up to a maximum of five
  tiers as required by the xHCI spec.  Port numbers greater than 15 are clamped to
  `0xF`.
- *Speed* — one of `SLOT_SPEED_SS/HS/FS/LS` is written into `dev_info`.
- *Root port* — `find_root_port` walks the parent chain to the root hub and records that
  port number in the `ROOT_HUB_PORT` field of `dev_info2`.
- *Transaction Translator info* — for low- or full-speed devices the driver walks the
  parent chain upward until it finds the nearest high-speed hub ancestor, then programs
  `TT_SLOT` with that hub's slot ID and `TT_PORT` with the downstream port number.
- *EP0 max packet size* — set to 512 for SuperSpeed, 64 for high-speed and
  full-speed (the correct value is confirmed after reading the device descriptor), and
  8 for low-speed.
- *Virtual address vs xHCI address* — the USB address on the wire is chosen by the
  xHCI controller and returned in the `ADDRESS_DEVICE` completion event; it is stored
  as `xhci_address` and is the address the hardware actually uses (e.g., in
  `CLEAR_TT_BUFFER` requests to parent hubs).  The address the stack assigns via
  `SET_ADDRESS` is tracked separately in `virtual_address` and used only as a lookup key
  for the internal device context.  The two values are independent and will generally
  differ.

**Endpoint context (`CONFIGURE_ENDPOINT`).** When the stack issues `SET_CONFIGURATION`
or `SET_INTERFACE`, the driver builds one xHCI endpoint context for each active endpoint:

- *DCI index* — `xhci_get_ep_index` maps USB endpoint addresses to xHCI Doorbell Context
  Indices: `epnum × 2` for the control endpoint, `epnum × 2 − (IN ? 0 : 1)` for all
  others.  The highest active DCI becomes the `LAST_CTX` field in the slot context.
  `xhci_collect_config_masks` simultaneously builds the `ADD_FLAGS` bitmask for the
  `CONFIGURE_ENDPOINT` command.
- *Polling interval* — `xhci_get_endpoint_interval` converts `bInterval` into xHCI's
  power-of-two exponent `Interval` field using different rules per speed and transfer
  type.  High-speed bulk/control: `log₂(bInterval)` raw microframe count, clamped to
  0–15.  High-speed and SuperSpeed periodic (interrupt/isoc): `bInterval − 1` (already
  an exponent in the `2^(n-1)` encoding).  Full-speed isoc: the same exponent formula,
  then add 3 to convert frames to microframes (`1 frame = 2³ µframes`).  Low-speed and
  full-speed interrupt: `bInterval` in frames multiplied by 8, expressed as a
  power-of-two exponent.
- *Max burst and ESIT payload* — for SuperSpeed endpoints the companion descriptor
  provides `bMaxBurst` directly; for high-speed periodic endpoints the two transaction-
  opportunity bits in the high byte of `wMaxPacketSize` encode `max_burst − 1`.  The
  maximum ESIT payload is `wBytesPerInterval` from the companion descriptor for
  SuperSpeed, or `wMaxPacketSize × max_burst` for high-speed periodic endpoints.

**Hub descriptor prefetch.** The xHCI slot context for a hub must contain the number of
downstream ports and, for high-speed hubs, the TT think time.  Those values are only
available in the hub class descriptor, which the stack does not fetch before
`SET_CONFIGURATION`.  The driver therefore intercepts the `SET_CONFIGURATION` IOReq for
any hub device, issues an internal `GET_DESCRIPTOR` for `USB_DT_SS_HUB` or `USB_DT_HUB`
on EP0, and stashes the original IOReq.  The `GET_DESCRIPTOR` completion handler
(`xhci_udev_handle_hub_prefetch`) caches the result in `udev->ss_hub_desc`, programs
the slot context accordingly, and then lets the original `SET_CONFIGURATION` proceed.

### Teardown

On `CloseDevice()` / `Expunge()` the unit task is stopped, all rings are freed, the
interrupt is removed and the PCIe controller is left in a quiescent state.

---

## Requirements

- AmigaOS 3.1 or later (Kickstart V39 minimum)
- Poseidon 4.5 or compatible USB stack
- [PiStorm32-lite](https://github.com/PiStorm/pistorm32-lite) with Raspberry Pi 4B or CM4
- Emu68 1.1 alpha.1 or later — required for MMU mapping of the PCIe BAR window into the lower 4 GB
- `gic400.library` — [emu68-gic400-library](https://github.com/rondoval/emu68-gic400-library)

---

## Building

Build dependencies (must be installed first):

| Package | Where | Purpose |
|---|---|---|
| `Emu68Common` | `emu68-common` | Pool allocators, shared utilities |
| `Emu68PCIe` | `emu68-pcie-library` | BCM2711 PCIe controller + bus enumeration |
| `GIC400` | `emu68-gic400-library` | ARM GIC-400 interrupt controller (MSI) |

```sh
cd build
make -j4
make install    # installs xhci.device into ./install/
```

Copy `install/xhci.device` to `DEVS:USBHardware/` on the Amiga.

---

## Repository layout

```
xhci.device/
  src/
    device.c                AmigaOS Device init/open/close/expunge
    device_beginio.c        BeginIO dispatcher
    device_abortio.c        AbortIO handler
    unit.c                  Unit lifecycle: PCIe init, hardware bring-up, IRQ setup
    unit_task.c             Per-unit task: event loop, watchdog timer, command dispatch
    unit_commands.c         USB command processing (control, bulk, interrupt, iso)
    irq.c                   MSI / INTx interrupt service routine
    xhci/
      xhci.c                xHCI controller init/reset
      xhci-ring.c           Transfer and command ring management
      xhci-context.c        Device/slot/endpoint context management
      xhci-events.c         Event ring processing
      xhci-commands.c       Command TRB submission and completion
      xhci-td.c             Transfer descriptor construction
      xhci-descriptors.c    USB descriptor parsing helpers
      xhci-root-hub.c       Root hub emulation (USB 2.0 and USB 3.0 descriptor sets)
      xhci-udev.c           USB device state, SS-to-HS translation, hub emulation
      xhci-endpoint.c       Endpoint open/close/reset
  include/
    device.h                XHCIDevice / XHCIUnit structs, device interface
    config.h                Compile-time tunables (stack size, timeouts, …)
```

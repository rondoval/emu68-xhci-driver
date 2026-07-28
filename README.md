# emu68-xhci-driver

> **Releases:** this component ships as part of the
> [emu68-driver-stack](https://github.com/rondoval/emu68-driver-stack) — the downloadable
> `.lha` and bundled documentation are published there. This repository is source-only
> and versioned via git tags.

**xhci.device** is an AmigaOS USB 2.0 / USB 3.0 host controller driver for the xHCI
controllers on the Raspberry Pi 4B and CM4, for use with PiStorm32-lite and
[Emu68](https://github.com/michalsc/Emu68). The xHCI code is derived from
[Das U-Boot](https://source.denx.de/u-boot/u-boot).

> **Which line do you need?** `xhci.device` **5.x** — this line — is the compatibility
> line: it speaks the classic Poseidon HCD ABI, so it drives **classic Poseidon 4.x** and
> also works with **Poseidon for AmigaOS 6.x**, which keeps that ABI alongside its own.
> On Poseidon 4.x this is your only option. On Poseidon 6.x prefer the driver's **6.x**
> line — through this one, SuperSpeed devices still go through the USB 2.0 emulation and
> run slower, whereas the 6.x line hands the stack real USB 3.0 devices and adds bulk
> streams. It is maintained on the
> [`context_release` branch](https://github.com/rondoval/emu68-xhci-driver/tree/context_release).

> **Upgrading from an older release?** See the *Upgrade notes* at the top of
> [RELEASE-NOTES.md](RELEASE-NOTES.md) — unit renumbering and the `bcmpcie.library`
> requirement.

## What you get

- **USB 3.0 devices on a USB 2.0 stack** — SuperSpeed devices, hubs and their
  descriptors are translated to what classic Poseidon understands, so a USB 3.0 drive or
  hub works on a stack that has no USB 3.0 support of its own. Devices are presented as
  high-speed.
- **All four transfer types** — control, bulk, interrupt and real-time isochronous, in
  both directions, at low, full, high and SuperSpeed.
- **Both of the Pi's USB paths** — the onboard OTG port and the PCIe VL805 (the four
  USB-A ports on a Pi 4B), each as its own unit, each with its own driver task.
- **Hub chains** — external USB 2.0 hubs including multi-TT, and USB 3.0 hubs through the
  SuperSpeed hub emulation.
- **Working devices** — keyboards, mice and other HID, thumb drives and other mass
  storage, and USB audio cards.
- **Link power management** — USB 3.0 U1/U2, USB 2.0 hardware LPM (L1) and Latency
  Tolerance Messaging, plus suspend with the correct ring-stop ordering.
- **ROM-able** — the driver contains no writable data sections.

## Requirements

- AmigaOS 3.1 or later (Kickstart V39 minimum).
- **Classic Poseidon 4.x**, or **Poseidon for AmigaOS 6.x** — this line works with both,
  though on 6.x the 6.x driver line is faster. See the note above.
- [PiStorm32-lite](https://github.com/PiStorm/pistorm32-lite) with a Raspberry Pi 4B or CM4.
- Emu68 1.1 alpha.1 or later — needed to map the PCIe BAR window into the lower 4 GB.
- `gic400.library` — [emu68-gic400-library](https://github.com/rondoval/emu68-gic400-library).
- `bcmpcie.library` — [emu68-pcie-library](https://github.com/rondoval/emu68-pcie-library) —
  **required for unit 1+ (the VL805 and any other PCIe controller)**.
- `otg_mode=1` in `config.txt` if you want to use the OTG port.

## Unit numbering

| Unit | Port |
|---|---|
| 0 | Onboard OTG port (Pi 4B and CM4) |
| 1+ | PCIe xHCI controllers, indexed from 1 |

On a stock Pi 4B, unit 0 is the OTG port and unit 1 is the VL805 with the four USB-A
ports. On a CM4, unit 1 exists only if you have attached a PCIe xHCI card.

## Known limitations

- **SuperSpeed (USB 3.0) is experimental**, hubs included.
- **Data corruption is possible in edge cases** — back up before heavy use.
- **Real-time isochronous audio may glitch**, and non-RT isochronous transfers are
  untested. AHI 4.x is not supported yet.
- **No USB 3.0 bulk streams** — the fast UAS path for mass storage is only in the 6.x
  line, so USB 3.0 storage is slower here even under Poseidon 6.x.
- **USB 3.0 link power management does nothing on the Pi 4B's own ports** — the VL805
  never enters U1/U2 by fixed firmware policy, exactly as under Linux. It works on the
  downstream links of external SuperSpeed hubs.

---

## For developers

### Layout

`xhci.device/src/` is the AmigaOS device and unit layer — device entry points, the
BeginIO/AbortIO dispatchers, unit lifecycle and hardware bring-up, the per-unit task and
its watchdog, and the interrupt service routine. `xhci.device/src/xhci/` is the xHCI core:
controller init, ring and TD management, device/slot/endpoint contexts, event and command
processing, descriptor parsing, link power management, and the translation layer that
makes USB 3.0 devices and hubs look like USB 2.0 ones to the stack (`xhci-root-hub.c`,
`xhci-hub.c`, `xhci-udev.c`). Headers live in `xhci.device/include/`.

### Building

Built through the **emu68-driver-stack** superproject, which supplies the Bebbo
cross-toolchain and the companion CMake packages (`emu68-common`, `emu68-pcie-library`,
`emu68-gic400-library`). From a superproject checkout:

```sh
./scripts/docker-build.sh --target emu68-xhci-driver-legacy   # no local toolchain needed
```

The build runs inside the toolchain container, so do not invoke `cmake` on the host. The
superproject installs this line under `Storage/` — `Storage/DEVS/USBHardware/xhci.device`
— so it does not collide with the 6.x line's device of the same name; the installer copies
whichever one you pick to `DEVS:USBHardware/`.

Debug backend: `EMU68_CONFIGURE_ARGS="-DEMU68_DEBUG_BACKEND=serial" ./scripts/docker-build.sh`
(`pistorm` default | `serial` | `off`), selected stack-wide via `emu68-common`. `serial`
links `debug.lib` and is not ROM-able.

## License

GPL-family, per file — see the SPDX headers and [LICENSE](LICENSE). The xHCI core is
derived from Das U-Boot.

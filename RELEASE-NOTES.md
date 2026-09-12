# Upgrade notes

Configuration-relevant changes across all releases, newest first:

* **5.x is the compatibility line.**  It speaks the classic Poseidon HCD ABI, so
  it drives **classic Poseidon 4.x** and also works with **Poseidon for AmigaOS
  6.x**, which retains that ABI.  On Poseidon 6.x prefer the driver's **6.x**
  line — it is faster there — maintained on the
  [`context_release` branch](https://github.com/rondoval/emu68-xhci-driver/tree/context_release).
* **From 4.4 or later:** no configuration changes are required.
* **From 3.x:** `bcmpcie.library` must be installed in `LIBS:` for PCIe-based
  units (unit 1+, VL805 on Pi 4B) to work.
* **From pre-3.x:** unit numbering differs.  Unit 0 was the VL805 (PCIe) and
  is now the onboard OTG port; unit 1 is now the VL805.  Update your USB
  stack configuration accordingly.


# Release notes — xhci.device 5.4

Changes since v5.3.

---

## Can be built into a custom Kickstart ROM

The driver now comes up during the Kickstart boot sequence rather than after
DOS, so a ROM image built with poseidon-backport has a USB keyboard, mouse
and drives live in the early boot menu — and can boot from a USB drive.
The 6.x line gained the same thing in 6.2; use whichever line matches your
USB stack.

Nothing changes for the normal `DEVS:USBHardware/xhci.device` installation.

---

## Build & tooling

- Hardcoded `-m68040` removed — it overrode the toolchain's `M68K_CPU`, so
  non-68040 builds produced 68040 code.

---

# Release notes — xhci.device 5.3

Changes since v5.2.

---

## The 5.x line now ships alongside a 6.x line

`xhci.device` is released in two lines — only one can be installed at a time, and
the Emu68 driver stack installer asks which — differing in the USB stack ABI they
speak:

* **5.x — this line.**  Speaks the classic Poseidon HCD ABI.  Required on
  **classic Poseidon 4.x**, and usable on **Poseidon for AmigaOS 6.x** too, since
  that stack keeps the classic ABI alongside its own.  SuperSpeed devices go
  through the driver's internal USB 3.0 ↔ USB 2.0 translation either way.
* **6.x** — speaks only the newer context HCD ABI, so it needs Poseidon for
  AmigaOS 6.x.  It hands the stack real USB 3.0 devices with no emulation and
  adds USB 3.0 bulk streams for mass storage.

If you run classic Poseidon nothing changes: stay on this line.  If you run
Poseidon 6.x, this line still works, but the 6.x line is the faster choice.  In
the stack archive the 5.x driver rides under `Storage/` and is copied to
`DEVS:USBHardware/` only when you pick it.

---

## Maintenance

No functional changes to the driver.  It is built against the current
`emu68-common` debug API so the 5.x line compiles in the present driver stack
again: the old two-level `DEBUG` / `DEBUG_HIGH` scheme becomes the cumulative
tier ladder (`off` / `profile` / `debug` / `trace`), and one switch fall-through
is now spelled with the attribute the current compiler expects.  The emitted
code and runtime behaviour are unchanged.


# Release notes — xhci.device 5.2

Changes since v5.1.

---

## Compatibility

### Returned device descriptors report USB 2.0 to match the presented speed

`xhci.device` presents SuperSpeed devices to the USB stack as high-speed and
handles the SuperSpeed specifics internally through its USB 3.0 ↔ USB 2.0
translation layer.  The device descriptor returned to the stack now has its
`bcdUSB` field clamped to `0x0210` whenever a device reports USB 3.0 or later
(`bcdUSB >= 0x0300`), so the advertised USB revision is consistent with that
high-speed presentation.

This matters for USB 3.0-aware stacks: one that reads `bcdUSB` would otherwise
see a SuperSpeed revision that contradicts the high-speed device it is handed,
and could mis-handle it.  Clamping the field keeps the translation transparent
to the stack above.

---

## Build & tooling

* The explicit `<exec/execbase.h>` and `<minlist.h>` includes added in 5.1 for
  older NDK headers have been dropped; the driver targets the NDK 3.2 headers.


# Release notes — xhci.device 5.1

Changes since v5.0.

---

## Breaking changes

None to the driver's interfaces — unit numbering and the Poseidon-compatible HCD
interface are unchanged.

The runtime dependency moves forward, though: 5.1 uses the typed, multi-vector
interrupt API and therefore **requires `bcmpcie.library` 2.0 or later** (it calls
`AllocIntVectors` and friends at LVOs -342…).  The driver now opens the library
requesting version 2, so it fails to start cleanly — rather than crashing — if
only an older 1.x library is installed.

---

## New features

### MSI-X interrupts

The driver now allocates its interrupt through `bcmpcie.library` 2.0's typed,
multi-vector API (`AllocIntVectors` → `AddIntVectorServer`), choosing the best
available type in the order **MSI-X → MSI → INTx**.  MSI-X is used whenever the
controller and the XHCI device support it; the old single-vector `EnableMSI` /
`pci_add_intserver` path has been replaced.  A new `DEVICE_USE_MSIX` build option
(default on) can forbid MSI-X, just as `DEVICE_USE_MSI` already could forbid MSI.

Interrupt acknowledgement was simplified to match.  The ISR acks `USBSTS.EINT`
and gates the interrupter (`IMAN`), which deasserts the source for MSI/MSI-X and
INTx alike, so the driver no longer performs any PCIe-config-level masking
(`MaskMSI` / `CheckSetINTxMask` are gone from the hot path).  The obsolete
`msi_enabled` controller field was removed, and interrupt-setup failures are now
logged with `pcie_strerror()` for a readable reason.

---

## Improvements

### Release builds drop all diagnostics

Every diagnostic helper is now gated behind `DEBUG`, so release builds compile it
out entirely: the slot / endpoint / config / caps / request dumps, the
endpoint default-state handler, the command- and state-name string helpers, and
the verbose PCI-config probe in device detection.  This shrinks the non-debug
binary and keeps it free of the unused-symbol warnings those helpers would
otherwise raise.  The stack-wide debug backend is selectable at build time
(`-DEMU68_DEBUG_BACKEND=pistorm|serial|off`, via `emu68-common`).

### NDK 3.9 / -O3 build portability

The driver builds cleanly under NDK 3.9 at `-O3` with `-Wconversion` /
`-Wsign-conversion`: the cache-flush helpers take `ULONG` lengths to match
`CachePreDMA` / `CachePostDMA`, the root-hub reply callback's actual-length
argument is `u32`, the stopped-ring dequeue pointer is carried as `dma_addr_t`,
`<exec/execbase.h>` is included explicitly for `DMA_ReadFromRAM`, and the
internal `mem_zero()` helper was replaced by `memset()` throughout.  No
functional change.

---

## Build & tooling

* The embedded `$VER:` string is now stamped `MAJOR.MINOR` (the patch component
  is dropped).
* A CI versioning / release-check workflow was added.


# Release notes — xhci.device 5.0

Changes since v4.4.

---

## Breaking changes

None.  Unit numbering, the `bcmpcie.library` runtime dependency and the
Poseidon-compatible HCD interface are unchanged from 4.4.  Existing USB stack
configurations continue to work without modification.

---

## New features

### USB 3.0 Link Power Management (U1/U2) and USB 2.0 hardware LPM (L1)

The driver now implements full Link Power Management.  For SuperSpeed devices it
computes the U1/U2 SEL/PEL/MEL latency parameters (USB 3.1 Appendix C), issues
`SET_SEL` to the device, programs the port U1/U2 timeouts and enables
device-initiated U1/U2 transitions via `SET_FEATURE(U1_ENABLE/U2_ENABLE)`.  The
Max Exit Latency is applied to the slot context through an Evaluate Context
command — the xHC only evaluates MEL on Address Device / Evaluate Context, so a
MEL carried in a Configure Endpoint context would be ignored.  The whole
sequence is kicked off only after the `SET_CONFIGURATION` control transfer
completes on the wire, because devices reject `U1/U2_ENABLE` until configured.

For USB 2.0 devices sitting directly on a root-hub port, hardware-controlled L1
(BESL) is enabled when both the host and the device advertise it: the driver
parses the device's BOS USB 2.0 Extension capability for BESL support and
baseline/deep values, computes the HIRD/BESL host value and programs the port
accordingly.

Note on the Raspberry Pi 4B's VL805: its root ports never enter U1/U2 under any
mechanism — this is a fixed firmware policy (confirmed identical under Linux),
not a driver limitation.  On the Pi 4B, USB 3.0 LPM therefore only takes effect
on the downstream links of external SuperSpeed hubs, which the driver programs
correctly.

### Latency Tolerance Messaging (LTM)

For configured SuperSpeed devices whose BOS descriptor advertises LTM, and when
the controller reports Latency Tolerance Messaging Capability, the driver now
enables LTM with `SET_FEATURE(LTM_ENABLE)`.  The resulting device→host LTM
packets are consumed by the xHC in hardware and feed its U-state timing.

### Multi-TT hub support

When a high-speed hub exposes the multiple-Transaction-Translator capability and
its multi-TT interface alternate setting is selected, the driver now sets the
`MTT` bit in the relevant xHCI slot contexts (the hub's own context, and the
context of any low-/full-speed device sitting behind it).  Previously the driver
always treated such hubs as single-TT.  This improves throughput and reliability
for low- and full-speed devices behind multi-TT hubs.

---

## Bug fixes

### Timed-out commands are now retired when the command ring is stopped

When a command was aborted because it exceeded the command-ring timeout, it was
possible for the Command Abort completion event to go missing.  The driver now
detects a timed-out head command on the Command Ring Stopped event and fails it
explicitly (returning it to the caller) instead of leaving it stuck on the
pending list.  It also reprograms `CRCR` to the next software enqueue position
so that the ring restarts on queued commands rather than re-executing the
aborted TRB.

### More accurate USB error reporting

Transaction errors are now reported with USB-correct error codes.  `ERR_TIMEOUT`
(no answer at all), `ERR_NAK_TIMEOUT` (request retired after the caller's
deadline, with a partial actual length) and `ERR_CRC_ERROR` (USB transaction /
CRC error) are now distinguished and surfaced to the stack.  This matters
because Poseidon's dead-device heuristic weighs these codes differently, so
correct classification avoids both premature device drops and missed ones; it
also lets bulk pipe streams treat a NAK timeout with non-zero actual length as
continuable partial success.

---

## Improvements

### Reset guard: controllers are halted before a machine reset

The driver now installs a reset guard that halts every xHCI controller — and
therefore stops all ring, event and MSI DMA — before the machine resets.  This
prevents in-flight DMA from corrupting memory or hanging the host across a
reboot.  If the ColdReboot vector has been re-patched on top of the driver's
stub, the device correctly refuses to expunge and stays resident so the guard
remains valid.

### Always warm-reset SuperSpeed root ports

SuperSpeed root-port resets are now always issued as warm resets rather than
only when the port is in the Compliance or SS.Inactive link state.  A hot reset
can leave a SuperSpeed link trained but dysfunctional (the port reports
enabled/U0, yet `ADDRESS_DEVICE` times out) in a way that is not observable from
`PORTSC`.  A warm reset forces full link retraining and is a strict superset of
a hot reset, so it is the reliable path for bringing up SuperSpeed devices.

### VL805 controller quirks

PCIe controllers are now probed for known quirks at PCI bring-up.  The VIA VL805
gets two workarounds: ring segments are allocated at double size because the
controller prefetches past segment ends (`TRB_OVERFETCH`), and max burst is
forced to 0 for SuperSpeed bulk-OUT endpoints of mass-storage devices behind a
hub (`SS_BULK_OUT`), which the VL805 otherwise mishandles.  Two spec-legal,
zero-cost recovery fixes are applied unconditionally for all controllers: the
Set TR Dequeue value never points at a link TRB, and recovery re-arms derive the
cycle bit from the stopped TRB rather than from the (sometimes wrongly written)
endpoint-context dequeue cycle state.  The onboard BCM2711 / CM4 controllers run
quirk-free.

### Memory-management refactor

Internal DMA allocation has been reworked to use page-bounded allocations that
respect the controller's reported page size, with consolidated DMA and metadata
pools and improved error handling.  Event-ring segment allocation now respects
the maximum the controller advertises.  This improves alignment correctness and
robustness with no change to externally visible behavior.

### Endpoint rings are stopped before U3 suspend

When a device is suspended (U3), the driver now stops the device's endpoint
rings before writing the port into U3, as required by xHCI §4.15.1, making
suspend/resume more robust.

### Code-structure refactor

SuperSpeed hub emulation and Link Power Management have been split out of
`xhci-udev.c` into dedicated `xhci-hub.c` and `xhci-lpm.c` modules, the RT
isochronous and ring-enqueue paths have been refactored, and a `driver_state`
field is now tracked per unit so `CMD_DEVICE_QUERY`/`TAG_DRIVER_STATE` reports
the unit's real operational/reset/suspended state instead of a hard-coded
value.  Device vendor/product queries now return friendly names ("Broadcom" /
"BCM2711 xHCI", "VIA Labs" / "VL805 xHCI").  These are internal changes with no
change to the on-wire or stack-facing protocol.


# Release notes — xhci.device 4.4

Changes since v3.7.

---

## Breaking changes

### `bcmpcie.library` is now a required runtime dependency

PCIe controller support (unit 1+, VL805 on Raspberry Pi 4B) previously used a
statically linked copy of the PCIe driver embedded inside `xhci.device`.  In 4.4 this
has been replaced by a dynamic dependency on `bcmpcie.library`.

**You must install `bcmpcie.library` in `LIBS:` before opening unit 1 or higher.**

`bcmpcie.library` is built from the
[emu68-pcie-library](https://github.com/rondoval/emu68-pcie-library) repository.

Unit 0 (onboard OTG port) is unaffected and does not require `bcmpcie.library`.

---

## Bug fixes

### Transfer ring full spin-loop (non-debug builds)

In builds without `DEBUG_HIGH`, `xhci_ep_schedule_next` could spin indefinitely when
all TRBs on an endpoint's transfer ring were occupied.  The full-ring path in
`xhci_ring_enqueue_td` re-queued the request to the head of the pending list and
returned `ERR_NO_ERROR`; the scheduler immediately dequeued and retried it, repeating
forever.

### ISO TRBs: TBC and TLBPC fields now populated

Isochronous TRBs were previously submitted with TBC and TLBPC left at zero.  Both 
fields are now calculated correctly from the transfer length and the endpoint's
`bMaxBurst` and `wBytesPerInterval` values, as required by the xHCI specification.

### ISO event-processing fixes

Completion-event handling for isochronous TDs has been tightened: stale residue values
are cleared before processing and the event-ring drain loop correctly handles
back-to-back completions without losing events.

### Interrupt coalescing enabled

The xHCI runtime interrupt registers are now programmed with interrupt moderation values
(`IMODI`/`IMODC`).  The U-Boot legacy code left these at zero (effectively disabling coalescing);
the driver now applies a small moderation interval to reduce IRQ rate under sustained
isochronous load.

### RT isochronous scheduling fixes

For RT IN transfers the driver calculates a per-endpoint scheduling horizon based on the
polling interval, avoiding over-scheduling that previously led to event ring congestion.
The ISP (Interrupt on Short Packet) flag is no longer set on isochronous rings, as it
can spuriously interrupt the host on partial packets that are normal for audio streams.
The RT slab cache (see below) is initialised once per endpoint rather than per device,
preventing cache sharing between endpoints with different object sizes.

### SPDX license identifiers corrected to `GPL-2.0-only`

All source and header files that previously carried `GPL-2.0+` have been corrected to
`GPL-2.0-only`.  The `+` suffix was incorrect given the provenance of the xHCI code
that was imported from Das U-Boot.

---

## Improvements

### Real-time isochronous transfers

The isochronous engine has been overhauled:

- **Per-controller scheduling mode.** CM4 controllers use Contiguous Frame ID scheduling;
  the VL805 continnues to use SIA flag.
- **Correct TRB types.** Each TRB in a multi-TRB TD is now assigned the correct type
  (`TRB_ISOC` for the first, `TRB_NORMAL` for continuations), fixing completion event
  matching on some controllers.
- **Slab allocation for TDs and `USBIORequest` objects.** Transfer descriptors and
  `USBIORequest` structures are allocated from per-endpoint slab caches (see below),
  eliminating per-transfer allocation overhead on the hot path.

### Growing transfer ring; 256 TRBs per segment

Transfer rings now grow on demand: when the scheduler finds a ring full, a new segment
of 256 TRBs is linked in rather than deferring the request.  The default event ring has
also been enlarged to match, reducing the probability of event ring overflow under peak
isochronous load.  The segment size constant (`TRBS_PER_SEGMENT`) is defined in
`config.h` and remains a compile-time knob.

### O(1) slab allocator

A new fixed-size O(1) slab allocator (`slab_cache`) from `emu68-common` replaces
pool-based allocation for all hot-path objects.  Each `slab_cache` pre-allocates a
contiguous DMA-aligned slab and serves objects in constant time with no per-object
header overhead.  In `xhci.device` three object types are now slab-allocated:

- Transfer descriptors (TDs)
- `USBIORequest` wrappers
- Real-time isochronous request structures

Each endpoint that carries isochronous traffic owns its own RT slab cache, sized to its
polling interval, so caches are never shared across endpoints with different object
sizes.  Non-isochronous endpoints share the per-unit TD and request caches.

### Ring backpressure

`xhci_ring_has_room()` is checked before enqueuing a transfer descriptor.  If the ring
is full the request is deferred; the growing ring logic then expands capacity before the
next attempt.  This replaces a spin that existed in the previous full-ring path.

### xHCI internal type cleanup

All internal xHCI types have been audited and aligned to the `u8`/`u16`/`u32`/`s8`/
`s32` typedefs shared with `emu68-common`, replacing the previous mix of `int`, `LONG`,
`ULONG` and AmigaOS types in low-level structures.  Affected fields include endpoint
context indices, max packet sizes, RT ISO frame counters, and inflight byte accounting.
The change has no runtime effect but improves portability and silences a number of
sign-conversion warnings.

### Aligned to updated `emu68-common` helpers

Internal ring management, event handling, descriptor walking, root-hub emulation and
device context setup have been updated to use the revised helper surface from
`emu68-common`.  Printf format strings using `%ld`/`%d` on fields that are now unsigned
have been corrected to `%lu`/`%u` to silence warnings.

### Reduced default log verbosity

Several high-frequency log statements (port polling, command-ring activity, transfer
completions) are now gated behind `DEBUG_HIGH`, reducing noise in production builds.


# Release notes — xhci.device 3.7

Changes since v2.0.

---

## New features

### Onboard OTG port support (Raspberry Pi 4B and CM4)

Unit numbering has changed.  Unit 0 is now always the onboard OTG xHCI
controller; unit 1 and above are PCIe controllers.  The onboard controller is
located via the Emu68 device tree (`/scb/xhci`).
The driver now detects if the onboard xHCI block has been disabled in firmware
and skips it cleanly.

Update Poseidon's hardware configuration for the new unit assignments before
upgrading.

### USB 3.0 hub support

Full USB 3.0 hub support has been added.  This includes:

- Correct slot context programming: route string, hub flag, port count and TT
  think time are now populated from the hub class descriptor fetched internally
  before `SET_CONFIGURATION` completes.
- `SET_HUB_DEPTH` issued to SuperSpeed hubs during configuration.
- Port status and change word translation from SS semantics to USB 2.0 semantics,
  so the stack can manage SS hub ports normally.
- USB 2.0 hub requests (feature codes `SUSPEND`/`RESUME`, `ENABLE`/`C_ENABLE`,
  `C_SUSPEND`) translated on the fly to their SuperSpeed equivalents or silently
  swallowed where no SS equivalent exists.

### SuperSpeed (USB 3.0) device support

USB 3.0 devices connected to SS ports are now enumerated and operated at
SuperSpeed.  SuperSpeed Endpoint Companion descriptors from the configuration
descriptor are parsed internally and used to program xHCI endpoint contexts
(max burst, ESIT payload); they are stripped before the descriptor is returned
to the stack.  The device is presented to the stack as high-speed.

### Command ring timeouts and abort recovery

A 5-second timeout is enforced on all command ring commands.  `AbortIO()`
now posts an internal abort request to the unit task rather than attempting
teardown from caller context, making abort safe regardless of whether the
transfer is queued or already on the hardware ring.  In-flight TDs can be
cleanly cancelled via `STOP_RING`: their TRBs are patched to NOOP and the
transfer ring dequeue pointer is restored to the correct restart position.

---

## Improvements

- **Dual virtual root hub**: the virtual root hub now advertises as USB 2.0 when
  all ports are USB 2.0, and as USB 3.0 as soon as any SS port is present.
  Warm reset is used for USB 3.0 device attach.
- **USB 2.0 suspend/resume**: port link state transitions for USB 2.0 ports
  (U0 ↔ U3) are now handled correctly through the root hub state machine.
- **Standby on USB 3.0 ports**: `SUSPEND`/`RESUME` feature requests to SS hub
  ports translate to the correct `LINK_STATE` values (U3/U0).
- **Port teardown on disable**: devices are disconnected and their slots freed
  when a downstream port is disabled.
- **`CMD_FLUSH` fixes**: the root hub interrupt request is now properly aborted
  during `CMD_FLUSH`; the maximum endpoint index scan was off-by-one and is
  corrected.
- **RT isochronous audio glitch reduction**: a missing NULL guard on the ring's
  `deferred_giveback` pointer could cause a spurious giveback call, occasionally 
  producing audio glitches. The giveback is now skipped when the pointer is NULL
   and cleared afterward.
- **Hub resume duplicate request workaround**: during hub resume, the stack can
  re-submit the same interrupt `IORequest` while the original is still active on
  the endpoint.  The driver now detects this by checking whether the exact request
  object is already tracked on that endpoint and silently treats the duplicate
  send as a no-op, preventing the request from being queued or replied to twice.
- **Command ring stability**: uninitialized `pending_commands` list fixed.

---

## Compatibility

- **AmigaOS 3.1 (Kickstart V39)**: the driver no longer uses any API newer than
  V39.  It is now compatible with stock Kickstart 3.1 ROMs.
- Build system and API header cleaned up; internal debug output reduced.


**Full Changelog**: https://github.com/rondoval/emu68-xhci-driver/compare/v2.0...v3.7


# Release notes — xhci.device 3.6

## What's Changed
* CM4 support, USB 3.0 support and more by @rondoval in https://github.com/rondoval/emu68-xhci-driver/pull/12

I'll post a more detailed description later on.

**Breaking change**
The USB-A ports on the PI4 are now **unit 1**
That's because **unit 0** is the OTG port.



**Full Changelog**: https://github.com/rondoval/emu68-xhci-driver/compare/v2.0...v3.6


# Release notes — xhci.device 2.0

## What's Changed
* Fixed root hub status reporting to indicate self-powered operation
* Initial SuperSpeed support
* Stripping of NUL characters from product names of cheapo sound cards - these were displayed as '??????????' by Poseidon

SuperSpeed support is limited to devices directly connected to USB 3.0 ports on RPI; tested with a pendrive only.
USB3.0 hubs connected to USB 3.0 ports are not yet supported.
Connecting USB 2.0 devices to USB 3.0 ports is supported - it actually was in the previous version.
Since we can't report the true speed of 3.0 devices to Poseidon, the driver is adding " (SS)" to their names. This indicates the link to the device is actually SuperSpeed.


**Full Changelog**: https://github.com/rondoval/emu68-xhci-driver/compare/v1.7...v2.0


# Release notes — xhci.device 1.7

Let's call this a pre-release, early version.
May contain severe bugs. On the other hand, bug reports are welcome.

**Full Changelog**: https://github.com/rondoval/emu68-xhci-driver/commits/v1.7

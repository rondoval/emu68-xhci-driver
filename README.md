# emu68-xhci-driver

> **Releases:** this component ships as part of the
> [emu68-driver-stack](https://github.com/rondoval/emu68-driver-stack) — the downloadable
> `.lha` and bundled documentation are published there. This repository is source-only
> and versioned via git tags.

**xhci.device** is an AmigaOS USB 2.0 / USB 3.0 host controller driver for the xHCI
controllers on the Raspberry Pi 4B and CM4, for use with PiStorm32-lite and
[Emu68](https://github.com/michalsc/Emu68). The xHCI code is derived from
[Das U-Boot](https://source.denx.de/u-boot/u-boot).

> **This driver needs the new Poseidon.** `xhci.device` **6.x** talks to
> **Poseidon for AmigaOS 6.x** and nothing else: it implements only that stack's context
> HCD ABI, and answers the legacy per-transfer commands with `IOERR_NOCMD`. It will
> **not** work with classic Poseidon **4.x** (Chris Hodges) or the AROS **5.x** line.
> Those stacks want the driver's **5.x** line, which speaks the classic Poseidon HCD ABI
> and is maintained on the
> [`main` branch](https://github.com/rondoval/emu68-xhci-driver/tree/main) — the two
> lines are alternatives, so pick the one that matches your USB stack.

> **Upgrading from an older release?** See the *Upgrade notes* at the top of
> [RELEASE-NOTES.md](RELEASE-NOTES.md) — unit renumbering and the `bcmpcie.library`
> requirement.

## What you get

- **Real USB 3.0, not emulated** — SuperSpeed devices reach the stack as what they are,
  with their own descriptors and a real SuperSpeed root hub. The 5.x line had to disguise
  them as USB 2.0 devices and translate; Poseidon 6.x handles USB 3.0 itself, so that
  layer is gone.
- **Bulk streams** — the fast UAS path for mass storage: each queued command gets its own
  transfer ring, so a modern USB 3.0 drive is not held to one command at a time.
- **All four transfer types** — control, bulk, interrupt and real-time isochronous, in
  both directions, at low, full, high and SuperSpeed.
- **Both of the Pi's USB paths** — the onboard OTG port and the PCIe VL805 (the four
  USB-A ports on a Pi 4B), each as its own unit, each with its own driver task.
- **Hub chains** — external USB 2.0 hubs including multi-TT, and USB 3.0 SuperSpeed hubs.
- **Working devices** — keyboards, mice and other HID, thumb drives and other mass
  storage, and USB audio cards.
- **Link power management** — USB 3.0 U1/U2, USB 2.0 hardware LPM (L1) and Latency
  Tolerance Messaging, plus suspend/resume with device-initiated wake.
- **Low-overhead transfers** — MSI interrupts, and transfers issued straight from the
  calling task rather than through a message port.
- **ROM-able** — the driver contains no writable data sections.

## Requirements

- AmigaOS 3.1 or later (Kickstart V39 minimum).
- **Poseidon for AmigaOS 6.x** — see the note above; earlier Poseidon lines will not work.
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

- **Data corruption is possible in edge cases** — back up before heavy use.
- **USB 3.0 link power management does nothing on the Pi 4B's own ports** — the VL805
  never enters U1/U2 by fixed firmware policy, exactly as under Linux. It works on the
  downstream links of external SuperSpeed hubs.

---

## For developers

### Layout

`xhci.device/src/` is the AmigaOS device and unit layer — device entry points, unit
lifecycle and hardware bring-up, the per-unit task and its watchdog, and the interrupt
service routine. `xhci.device/src/xhci/` is the xHCI core: controller init, ring
mechanics and TD submission, device/slot/endpoint contexts, the context-ABI ingress and
direct transfer path, event and command processing, root-hub emulation, link power
management and the endpoint state machine. Headers live in `xhci.device/include/`.

### Building

Built through the **emu68-driver-stack** superproject, which supplies the Bebbo
cross-toolchain and the companion CMake packages (`emu68-common`, `emu68-pcie-library`,
`emu68-gic400-library`). From a superproject checkout:

```sh
./scripts/docker-build.sh --target emu68-xhci-driver   # no local toolchain needed
```

The build runs inside the toolchain container, so do not invoke `cmake` on the host. The
installed binary lands in `DEVS/USBHardware/xhci.device` under the install prefix.

Debug backend: `EMU68_CONFIGURE_ARGS="-DEMU68_DEBUG_BACKEND=serial" ./scripts/docker-build.sh`
(`pistorm` default | `serial` | `off`), selected stack-wide via `emu68-common`. `serial`
links `debug.lib` and is not ROM-able.

## License

GPL-family, per file — see the SPDX headers and [LICENSE](LICENSE). The xHCI core is
derived from Das U-Boot.

# emu68-xhci-driver — agent notes

## Build

This repo is a submodule of the `emu68-driver-stack` superbuild, at
`components/emu68-xhci-driver`. Build only through the stack's container
wrapper — never host `cmake`, since build trees are configured at `/work`
inside the toolchain container:

```sh
cd ../..    # emu68-driver-stack root
./scripts/docker-build.sh --target emu68-xhci-driver
```

- The prefix must already carry `emu68-common` and `emu68-pcie-library`; gic400
  headers arrive transitively via `Emu68PCIe::pcie_headers`, and
  `gic400.library` must be present at runtime.
- Debug backend: `EMU68_CONFIGURE_ARGS="-DEMU68_DEBUG_BACKEND=serial"` (`pistorm`
  default | `serial` | `off`), chosen stack-wide via `emu68-common`. `serial`
  links `debug.lib` and is not ROM-able.
- Target `emu68-xhci-driver` → `xhci.device` → `DEVS/USBHardware/xhci.device`.
- The driver is ROM-able: the link fails on any writable data section, so no
  mutable globals.
- Always build after C changes; it must come back clean of warnings.

## Layout

`xhci.device/src/` is the device/unit layer, `src/xhci/` the xHCI core,
`include/` the headers.

## Architecture

- **Unit 0** is the onboard OTG port (Pi 4B / CM4); **unit 1+** are PCIe xHCI
  controllers (VL805 etc.).
- The unit task polls at 100 ms (`UNIT_TASK_POLL_DELAY_MS`); command-ring
  operations time out at 5 s (`CMD_TIMEOUT_MS`). The ISR only signals the task —
  event-ring draining and command completion are task context.
- **Root hubs:** two protocol-pure emulations, SuperSpeed and USB2, addressed by
  the reserved context handles. The stack sees wire-truth descriptors.
- **Device addressing:** devices are keyed by the opaque context handle (== xHCI
  slot id); `xhci_address` is the wire address from `ADDRESS_DEVICE`. The driver
  never sends `SET_ADDRESS`.

## Poseidon HCD ABI (matched pair)

This driver and the `poseidon-backport` stack evolve together. That repo's
`docs/poseidon-context-hcd-abi.md` governs the ABI (`NSCMD_*` lifecycle ops,
`UHCF_CONTEXT` = BIT(5), `UHIOERR_NO_BANDWIDTH` = 14), alongside
`poseidon-vs-xhci-driver-model.md` and `implementation-plan.md`.

- The context ABI is the **only** client ABI; legacy `CMD_REQUEST_*` and classic
  RT-ISO commands reply `IOERR_NOCMD`.
- Internal request currency is `struct xhci_xfer` (`include/xhci/xhci-xfer.h`;
  `MinNode` first, completion via `xf->complete`). Transfers arrive as **direct
  calls** (`xhci-direct.c`: `NSCMD_USB_ATTACH` handshake, packed generation
  tokens from the create/configure ops); only lifecycle IOStdReq ops and bus
  commands travel as messages.
- Value-level contracts: `ERR_*` values and their dead-device weighting
  (TIMEOUT +3 / NAK_TIMEOUT +2 / CRC +1), `CMD_FLUSH` reply-everything,
  endpoint-layer clear-halt dedup.

## Code handling

- `unit_task.c` owns ongoing controller work, watchdog handling, command-ring
  submission and `pending_commands` mutation; do not issue xHCI recovery
  commands from arbitrary caller context when that path exists.
- Internal helper messages to UnitTask come from ONE pool — `ctrl->metaPool`,
  with the rest of the transfer machinery. Pool access from caller context only
  under `ctrl->xfer_lock`.
- `AbortIO()` is a wish-decline no-op; nothing message-framed is abortable.
  Direct transfers abort through `xhci_direct_abort` under `ctrl->xfer_lock`.
  Never touch command or transfer rings from caller context outside that lock.
- `src/xhci/xhci-root-hub.c` is the stack-facing contract for root ports;
  preserve behaviour unless the task is about hub semantics.
- Prefer targeted fixes inside `src/xhci/` over reshaping the public device
  layer.
- Preserve the GPL-family SPDX headers (audited against Linux/U-Boot
  provenance); do not broaden the licence without new provenance evidence.

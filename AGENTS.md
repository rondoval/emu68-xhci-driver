# emu68-xhci-driver Agent Notes

## Build

This repo is a submodule of the `emu68-driver-stack` superbuild, at
`components/emu68-xhci-driver-legacy`. Build only through the stack's container
wrapper — never host `cmake`, since build trees are configured at `/work` inside
the toolchain container:

```sh
cd ../..    # emu68-driver-stack root
./scripts/docker-build.sh --target emu68-xhci-driver-legacy
```

- The prefix must already carry `emu68-common` and `emu68-pcie-library`; gic400 headers arrive transitively via `Emu68PCIe::pcie_headers`, and `gic400.library` must be present at runtime.
- Debug backend: `EMU68_CONFIGURE_ARGS="-DEMU68_DEBUG_BACKEND=serial"` (default `pistorm` | `serial` | `off`); selected stack-wide via `emu68-common`, `serial` links `debug.lib` and is not ROM-able.
- The stack installs this flavor under `Storage/`, so the driver lands in `install/Storage/DEVS/USBHardware/xhci.device` — beside, not on top of, the context flavor's `install/DEVS/USBHardware/xhci.device`.

## Code Handling

- Treat `unit_task.c` as the owner of ongoing controller work, watchdog handling, and queued command submission.
- Treat UnitTask as the owner of command-ring submission and `pending_commands` mutation.
- Do not issue xHCI recovery commands directly from arbitrary caller context when the UnitTask path exists.
- Internal helper messages posted to UnitTask must be allocated from `unit->memoryPool`, not `ctrl->memoryPool`.
- Safe `AbortIO()` behavior is to mark and defer hardware teardown work to UnitTask rather than manipulating command or transfer rings directly from caller context.
- Root-hub emulation and descriptor translation in `xhci-root-hub.c` and `xhci-udev.c` are compatibility glue for Poseidon; preserve behavior unless the task is explicitly about hub semantics.
- Prefer targeted fixes in `xhci/` internals over broad reshaping of the public device-layer code.
- Licensing in this repo was audited against Linux/U-Boot provenance; preserve the current GPL-family SPDX headers and do not reintroduce broader dual-license wording without new provenance evidence.

## Validation

- Check Problems on changed files first.
- If changes touch shared interfaces or build outputs, validate through `emu68-driver-stack`.
- If changes are local to the driver, `./scripts/docker-build.sh --target emu68-xhci-driver-legacy` from the stack root is enough.
- Keep the internal notes in `README-internal.md` in mind for timeout, abort, and queue-depth related work, but do not treat them as a design spec.


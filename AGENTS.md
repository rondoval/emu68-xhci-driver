# emu68-xhci-driver Agent Notes

## Build

- Required installed dependencies: `emu68-common`, `emu68-pcie-library`, and `emu68-gic400-library`.
- Preferred commands:
  - `cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain.cmake -DCMAKE_PREFIX_PATH=/path/to/emu68-driver-stack -DCMAKE_INSTALL_PREFIX=/path/to/emu68-driver-stack`
  - `cmake --build build`
  - `cmake --install build`
- The installed binary goes to `DEVS/USBHardware/xhci.device` under the selected prefix.

## Code Handling

- Treat `unit_task.c` as the owner of ongoing controller work, watchdog handling, and queued command submission.
- Treat UnitTask as the owner of command-ring submission and `pending_commands` mutation.
- Do not issue xHCI recovery commands directly from arbitrary caller context when the UnitTask path exists.
- Internal helper messages posted to UnitTask must be allocated from `unit->memoryPool`, not `ctrl->memoryPool`.
- Safe `AbortIO()` behavior is to mark and defer hardware teardown work to UnitTask rather than manipulating command or transfer rings directly from caller context.
- Root-hub emulation and descriptor translation in `xhci-root-hub.c` and `xhci-udev.c` are compatibility glue for Poseidon; preserve behavior unless the task is explicitly about hub semantics.
- Prefer targeted fixes in `xhci/` internals over broad reshaping of the public device-layer code.

## Validation

- Check Problems on changed files first.
- If changes touch shared interfaces or build outputs, validate through `emu68-driver-stack`.
- If changes are local to the driver, a repo-local build is appropriate once `build/` has been configured.
- Keep the internal notes in `README-internal.md` in mind for timeout, abort, and queue-depth related work, but do not treat them as a design spec.


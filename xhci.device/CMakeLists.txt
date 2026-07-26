cmake_minimum_required(VERSION 3.14.0)
project(xhci.device VERSION ${CMAKE_PROJECT_VERSION})

# Toolchain and compile options
add_link_options(
    -nostdlib
    -nostartfiles
    -s
    -Wl,-e,_doNotExecute
)

add_compile_options(
    -O3
    -m68040
    -fomit-frame-pointer
    -ffreestanding
    -Wall
    -Wextra
    -Wconversion
    -Wsign-conversion
    -Wshadow
    -Wmissing-prototypes
    -Wstrict-prototypes
)

# Debug-output backend defines (EMU68_DEBUG_BACKEND, see emu68-common).
emu68_debug_definitions()

# cache_ops.h LVO-fallback define (EMU68_FORCE_LVO_CACHE_OPS, see emu68-common).
emu68_cache_ops_definitions()

add_compile_definitions(
    PRIVATE
        DEVICE_IDSTRING="${VERSTRING}"
        DEVICE_VERSION=${PROJECT_VERSION_MAJOR}
        DEVICE_REVISION=${PROJECT_VERSION_MINOR}
        DEVICE_NAME="${PROJECT_NAME}"
        DEVICE_USE_MSI=TRUE
)

# Sources
add_executable(xhci.device
    src/device.c
    src/unit.c
    src/unit_task.c
    src/irq.c
    src/unit_commands.c
    src/xhci/xhci.c
    src/xhci/xhci-ring.c
    src/xhci/xhci-submit.c
    src/xhci/xhci-context.c
    src/xhci/xhci-lpm.c
    src/xhci/xhci-events.c
    src/xhci/xhci-commands.c
    src/xhci/xhci-td.c
    src/xhci/xhci-root-hub.c
    src/xhci/xhci-udev.c
    src/xhci/xhci-endpoint.c
    src/xhci/xhci-ep-rtiso.c
    src/xhci/xhci-ctx-ops.c
    src/xhci/xhci-direct.c
    src/device_end.c
)

target_include_directories(xhci.device
    PRIVATE
        include
)

target_link_libraries(xhci.device
    PRIVATE
        Emu68Common::common
        Emu68PCIe::pcie_headers
)

# ROM-able: no writable .data/.bss may sneak in (see driver-stack TODO.md). For the
# serial backend this instead links debug.lib (+ __divsi3 glue) and skips the check.
emu68_debug_backend_finalize(xhci.device ROMABLE)

install(
    PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/xhci.device
    DESTINATION DEVS/USBHardware
)

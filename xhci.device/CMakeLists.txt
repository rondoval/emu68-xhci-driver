cmake_minimum_required(VERSION 3.14.0)
project(xhci.device VERSION ${CMAKE_PROJECT_VERSION})

# Toolchain and compile options
add_link_options(
    -ffreestanding
    -nostdlib
    -nostartfiles
    -s
    -Wl,-e,_doNotExecute
)

add_compile_options(
    -Os
    -m68040
    -Wall
    -Wextra
    -Wconversion
    -Wsign-conversion
    -Wshadow
    -Wmissing-prototypes
    -Wstrict-prototypes
    -DDEBUG
    -fomit-frame-pointer
)

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
    src/device_beginio.c
    src/device_abortio.c
    src/unit.c
    src/unit_task.c
    src/irq.c
    src/unit_commands.c
    src/xhci/xhci.c
    src/xhci/xhci-ring.c
    src/xhci/xhci-context.c
    src/xhci/xhci-lpm.c
    src/xhci/xhci-hub.c
    src/xhci/xhci-events.c
    src/xhci/xhci-commands.c
    src/xhci/xhci-td.c
    src/xhci/xhci-descriptors.c
    src/xhci/xhci-root-hub.c
    src/xhci/xhci-udev.c
    src/xhci/xhci-endpoint.c
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

# ROM-able: no writable .data/.bss may sneak in (see driver-stack TODO.md)
emu68_rom_check(xhci.device)

install(
    PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/xhci.device
    DESTINATION DEVS/USBHardware
)

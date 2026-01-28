cmake_minimum_required(VERSION 3.14.0)
project(xhci.device VERSION ${CMAKE_PROJECT_VERSION})

add_link_options(-ffreestanding -nostdlib -nostartfiles -s -Wl,-e,_doNotExecute)
add_compile_options(-Os -m68040 -Wall -Wextra -DDEBUG -fomit-frame-pointer)
add_compile_definitions(PRIVATE DEVICE_IDSTRING="${VERSTRING}" DEVICE_VERSION=${PROJECT_VERSION_MAJOR} DEVICE_REVISION=${PROJECT_VERSION_MINOR} DEVICE_NAME="${PROJECT_NAME}")

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
    src/xhci/xhci-debug.c
    src/xhci/xhci-mem.c
    src/xhci/xhci-events.c
    src/xhci/xhci-commands.c
    src/xhci/xhci-td.c
    src/xhci/xhci-usb.c
    src/xhci/xhci-root-hub.c
    src/xhci/xhci-udev.c
    src/xhci/xhci-endpoint.c
    src/device_end.c
)

target_include_directories(xhci.device PRIVATE include)
target_link_libraries(xhci.device
    PRIVATE
        Emu68Common::common
        Emu68PCIe::pcie
)

install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/xhci.device DESTINATION .)

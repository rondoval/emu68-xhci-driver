cmake_minimum_required(VERSION 3.14.0)
project(xhci-tools VERSION ${CMAKE_PROJECT_VERSION})

set(XHCI_TOOL_COMPILE_OPTIONS
    -Os
    -Wall
    -Wconversion
    -Wsign-conversion
    -Wshadow
    -Wmissing-prototypes
    -Wstrict-prototypes
)

set(XHCI_TOOL_LINK_OPTIONS
    -s
)

add_executable(diskbench
    diskbench.c
    diskbench_util.c
    diskbench_fs.c
    diskbench_dev.c
)

target_compile_options(diskbench PRIVATE ${XHCI_TOOL_COMPILE_OPTIONS})

target_compile_definitions(diskbench PRIVATE
    VERSTAG="$VER: diskbench ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR} ${CURRENT_DATE}"
)

target_link_options(diskbench PRIVATE ${XHCI_TOOL_LINK_OPTIONS})

# diskbench is a developer benchmark — built for local HW testing but deliberately
# not installed, so it stays out of the shipped package.

cmake_minimum_required(VERSION 3.14.0)
project(emu68-xhci-driver VERSION 5.6)

string(TIMESTAMP CURRENT_DATE "(%d.%m.%Y)")
if(EMU68_BUILD_STAMP)   # local-build marker from the stack's build.sh; empty in releases
    string(APPEND CURRENT_DATE " ${EMU68_BUILD_STAMP}")
endif()
set(VERSTRING "$VER: ${PROJECT_NAME} ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR} ${CURRENT_DATE}")

find_package(Emu68Common 2.1 REQUIRED) # 2.1: DT_OpenSymbol
find_package(Emu68PCIe 2.0 REQUIRED) # 2.0: the multi-vector interrupt API

# Set default install prefix to ./install subdirectory
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/install" CACHE PATH "Install path" FORCE)
endif()

add_subdirectory(xhci.device)

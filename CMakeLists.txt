cmake_minimum_required(VERSION 3.14.0)
project(emu68-xhci-driver VERSION 0.1)

string(TIMESTAMP CURRENT_DATE "(%d.%m.%Y)")
set(VERSTRING "$VER: ${PROJECT_NAME} ${PROJECT_VERSION} ${CURRENT_DATE}")

# Set default install prefix to ./install subdirectory
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/install" CACHE PATH "Install path" FORCE)
endif()

add_subdirectory(common)
add_subdirectory(pcie-lib/pcie.library)
add_subdirectory(xhci.device)

# install(FILES ${CMAKE_SOURCE_DIR}/Install DESTINATION .)
# install(FILES ${CMAKE_SOURCE_DIR}/Install.info DESTINATION .)

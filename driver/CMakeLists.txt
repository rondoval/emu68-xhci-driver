cmake_minimum_required(VERSION 3.14.0)
project(xhci_driver VERSION 0.1.0)

add_compile_options(-Os -m68040 -fomit-frame-pointer -Wall -Wno-unused-function -Wno-shift-count-overflow -DDEBUG)
file(GLOB_RECURSE xhci_sources src/*.c include/*.h)
add_library(xhci_driver STATIC ${xhci_sources})

target_include_directories(xhci_driver PUBLIC include)
target_link_libraries(xhci_driver common pcie.library)

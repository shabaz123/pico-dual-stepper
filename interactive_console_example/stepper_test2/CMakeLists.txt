cmake_minimum_required(VERSION 3.28)
set(PICO_BOARD pico_w)
include(pico_sdk_import.cmake)

project(stepper_test2 C CXX ASM)

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmodules-ts -fcommon -fno-rtti -fno-exceptions")

pico_sdk_init()

include(FetchContent)

FetchContent_Declare(stepper
  GIT_REPOSITORY "https://github.com/jancumps/pio_stepper_lib.git"
  GIT_TAG "origin/main"
)
FetchContent_MakeAvailable(stepper)


add_executable(${CMAKE_PROJECT_NAME}
        source/TB67S581.cpp
        source/TB67S581.h
        source/console.cpp
        source/console.h)
target_sources(${CMAKE_PROJECT_NAME}
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/source/main.cpp
)
target_sources(${CMAKE_PROJECT_NAME}
        PUBLIC
        FILE_SET cxx_modules TYPE CXX_MODULES FILES
)

target_link_libraries( ${CMAKE_PROJECT_NAME}
        pico_stdlib
        hardware_gpio
        hardware_i2c
        pico_cyw43_arch_none
        stepper
)

# adjust to enable stdio via usb, or uart
pico_enable_stdio_usb(${CMAKE_PROJECT_NAME} 1)
pico_enable_stdio_uart(${CMAKE_PROJECT_NAME} 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(${CMAKE_PROJECT_NAME})

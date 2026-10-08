cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_BOARD pico_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

# usbin.c is written against SDK 1.5.1 with its lib/tinyusb at 0.17.0 (README.md).
if (NOT PICO_SDK_VERSION_STRING VERSION_EQUAL "1.5.1")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.5.1 required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(hid-bridge C CXX ASM)

pico_sdk_init()

add_executable(hid-bridge hid-bridge.c usbin.c)

target_link_options(hid-bridge PRIVATE -Xlinker --print-memory-usage)
target_compile_options(hid-bridge PRIVATE -Wall -Wextra)

add_compile_definitions(PICO_PANIC_FUNCTION=reset)

pico_set_program_name(hid-bridge "hid-bridge")
pico_set_program_version(hid-bridge "1.0")

pico_enable_stdio_uart(hid-bridge 1)
pico_enable_stdio_usb(hid-bridge 0)

target_include_directories(hid-bridge PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(hid-bridge pico_stdlib tinyusb_host tinyusb_board)

pico_add_extra_outputs(hid-bridge)

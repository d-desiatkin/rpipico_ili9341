cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(ILI9341_SKUMSP2402_DISPLAY CXX C ASM)
set(PICO_BOARD waveshare_rp2040_zero)
set(PICO_CXX_ENABLE_EXCEPTIONS 1)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# rest of your project
add_subdirectory(test_gpio)
add_subdirectory(test_pio)
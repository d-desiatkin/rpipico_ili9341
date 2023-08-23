cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(ILI9341_test CXX C ASM)
set(PICO_BOARD waveshare_rp2040_zero)
set(PICO_CXX_ENABLE_EXCEPTIONS 1)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# rest of your project

# add_library(vector
#         MODULE
#         vector.cpp)

# target_link_libraries(vector pico_stdlib)

if (TARGET tinyusb_device)
    add_executable(display_test
            display_test.cpp
            )

    # pull in common dependencies
    target_link_libraries(display_test pico_stdlib hardware_spi)

    # For debug purposes
    # enable usb output, disable uart output
    pico_enable_stdio_usb(display_test 1)
    pico_enable_stdio_uart(display_test 0)

    # create map/bin/hex/uf2 file etc.
    pico_add_extra_outputs(display_test)

    # add url via pico_set_program_url
    # example_auto_set_url(display_test)
elseif(PICO_ON_DEVICE)
    message(WARNING "not building display_test because TinyUSB submodule is not initialized in the SDK")
endif()
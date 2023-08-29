if (TARGET tinyusb_device)
    add_executable(display_test_gpio
            display_test_gpio.cpp
    )

    target_link_directories(display_test_gpio PUBLIC ${CMAKE_SOURCE_DIR}/ili9341)
    # pull in common dependencies
    target_link_libraries(display_test_gpio pico_stdlib hardware_spi)

    # For debug purposes
    # enable usb output, disable uart output
    pico_enable_stdio_usb(display_test_gpio 1)
    pico_enable_stdio_uart(display_test_gpio 0)

    # create map/bin/hex/uf2 file etc.
    pico_add_extra_outputs(display_test_gpio)

    # add url via pico_set_program_url
    # example_auto_set_url(display_test)
elseif(PICO_ON_DEVICE)
    message(WARNING "not building display_test_gpio because TinyUSB submodule is not initialized in the SDK")
endif()
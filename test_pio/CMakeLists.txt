if (TARGET tinyusb_device)
    add_executable(display_test_pio)

    pico_generate_pio_header(display_test_pio ${CMAKE_CURRENT_LIST_DIR}/display_spi.pio)
    include_directories(${CMAKE_SOURCE_DIR})
    target_sources(display_test_pio PRIVATE
            display_test_pio.cpp
            pio_display_spi.c
            pio_display_spi.h
            logic_analyser.c
            logic_analyser.h
            )

    target_link_directories(display_test_pio PUBLIC ${CMAKE_SOURCE_DIR})
    target_link_libraries(display_test_pio PRIVATE pico_stdlib hardware_pio hardware_dma)

    # For debug purposes
    # enable usb output, disable uart output
    pico_enable_stdio_usb(display_test_pio 1)
    pico_enable_stdio_uart(display_test_pio 0)

    pico_add_extra_outputs(display_test_pio)

elseif(PICO_ON_DEVICE)
    message(WARNING "not building display_test_pio because TinyUSB submodule is not initialized in the SDK")
endif()

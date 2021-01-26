add_executable(dmx16
        dmx16.c
        )

pico_generate_pio_header(dmx16 ${CMAKE_CURRENT_LIST_DIR}/dmx16.pio)

target_link_libraries(dmx16
        pico_stdlib
        pico_mem_ops
        hardware_dma
        hardware_irq
        hardware_pio
        )

# enable usb output, disable uart output
pico_enable_stdio_usb(dmx16 1)
pico_enable_stdio_uart(dmx16 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(dmx16)

# add url via pico_set_program_url
example_auto_set_url(dmx16)

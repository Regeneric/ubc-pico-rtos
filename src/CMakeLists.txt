add_executable(rtos
    main.c
)

target_link_libraries(rtos 
    pico_stdlib 
    pico_cyw43_arch_none 
    freertos
    # lwipallapps 
    # lwipcore
)

pico_enable_stdio_usb(rtos 1)
pico_enable_stdio_uart(rtos 0)

pico_add_extra_outputs(rtos)
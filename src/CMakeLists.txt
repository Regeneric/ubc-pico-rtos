add_executable(rtos
    main.c
)

target_link_libraries(rtos 
    freertos
    
    pico_stdlib 
    pico_multicore
    pico_binary_info

    hardware_gpio
    hardware_irq
    hardware_sync
    hardware_flash
    hardware_i2c

    # pico_cyw43_arch_none 
    # pico_lwipallapps 
    # pico_lwipcore
)

target_compile_definitions(rtos PRIVATE
    FREE_RTOS_KERNEL_SMP=0
    portSUPPORT_SMP=0
)

pico_enable_stdio_usb(rtos 1)
pico_enable_stdio_uart(rtos 0)

pico_add_extra_outputs(rtos)

# picotool for automated uC flashing
add_custom_command(TARGET rtos
    POST_BUILD
    COMMAND cd ${CMAKE_CURRENT_BINARY_DIR} && picotool load -f -x rtos.uf2
)
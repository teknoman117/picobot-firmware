cmake_minimum_required(VERSION 3.13)

add_executable(pibot-firmware
    main.cpp
    motors.cpp
    pico_stdio_transport.c
)

#pico_generate_pio_header(balancer
#    ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.pio
#)

target_include_directories(pibot-firmware PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
)

target_compile_definitions(pibot-firmware PRIVATE
    CFG_TUD_CDC_RX_BUFSIZE=512
    CFG_TUD_CDC_TX_BUFSIZE=512
    CFG_TUD_CDC_EP_BUFSIZE=512
    PICO_UART_ENABLE_CRLF_SUPPORT=0
    PICO_STDIO_ENABLE_CRLF_SUPPORT=0
    PICO_STDIO_DEFAULT_CRLF=0
    NO_SYS=0
)

target_link_libraries(pibot-firmware PRIVATE
    pico_stdlib
    hardware_pwm
    hardware_pio
    hardware_uart
    i2cdevlib
    FreeRTOS-Kernel-Heap4
    microros
)

pico_set_program_name(pibot-firmware "pibot-firmware")
pico_set_program_version(pibot-firmware "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_usb(pibot-firmware 1)
pico_enable_stdio_uart(pibot-firmware 0)
pico_enable_stdio_semihosting(pibot-firmware 0)

pico_add_extra_outputs(pibot-firmware)

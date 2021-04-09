cmake_minimum_required(VERSION 3.13)

# pico_sdk_import.cmake is copied from
# https://github.com/raspberrypi/pico-sdk/blob/master/external/pico_sdk_import.cmake
include(pico_sdk_import.cmake)

project(rp2040-dongle)

pico_sdk_init()

add_executable(rp2040-dongle
        dmahandler.c
        main.c
        hidinterface.c
        acminterface.c
        usb_descriptors.c
        log.cpp
        stdio_usb.c
        RF24/RF24.cpp
        RF24/utility/rp2/gpio.cpp
        RF24/utility/rp2/spi.cpp
        RF24Network/RF24Network.cpp
        RF24Mesh/RF24Mesh.cpp
        wireless.cpp
        )

pico_generate_pio_header(rp2040-dongle ${CMAKE_CURRENT_LIST_DIR}/tx16.pio)

target_include_directories(rp2040-dongle PRIVATE
	${CMAKE_CURRENT_LIST_DIR}
	${CMAKE_CURRENT_LIST_DIR}/RF24
	${CMAKE_CURRENT_LIST_DIR}/RF24Network
	)

target_link_libraries(rp2040-dongle
        pico_stdlib
        pico_unique_id
        pico_mem_ops
        tinyusb_device
        tinyusb_board
        hardware_dma
        hardware_irq
        hardware_pio
        hardware_timer
        hardware_gpio
        hardware_spi
        )

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(rp2040-dongle)

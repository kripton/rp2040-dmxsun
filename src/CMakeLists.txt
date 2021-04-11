cmake_minimum_required(VERSION 3.13)

# pico_sdk_import.cmake is copied from
# https://github.com/raspberrypi/pico-sdk/blob/master/external/pico_sdk_import.cmake
include(pico_sdk_import.cmake)

project(rp2040-dongle)

pico_sdk_init()

include(../lib/RF24/CMakeLists.txt)
include(../lib/RF24Network/CMakeLists.txt)
include(../lib/RF24Mesh/CMakeLists.txt)

add_executable(rp2040-dongle
        dmahandler.c
        main.c
        hidinterface.c
        acminterface.c
        usb_descriptors.c
        log.cpp
        stdio_usb.c
        wireless.cpp
        )

pico_generate_pio_header(rp2040-dongle ${CMAKE_CURRENT_LIST_DIR}/tx16.pio)

target_compile_definitions(rp2040-dongle PUBLIC PICO_DEFAULT_SPI_SCK_PIN=2)
target_compile_definitions(rp2040-dongle PUBLIC PICO_DEFAULT_SPI_TX_PIN=3)
target_compile_definitions(rp2040-dongle PUBLIC PICO_DEFAULT_SPI_RX_PIN=4)

## Config for RF24Network
# Pretend header file is already included so it doesn't re-define our values here
# Thus, make sure you define ALL values here and not in RF24Network_config.h
target_compile_definitions(rp2040-dongle PUBLIC __RF24NETWORK_CONFIG_H__)
target_compile_definitions(rp2040-dongle PUBLIC NETWORK_DEFAULT_ADDRESS=04444)
target_compile_definitions(rp2040-dongle PUBLIC RF24NetworkMulticast)
target_compile_definitions(rp2040-dongle PUBLIC MAX_PAYLOAD_SIZE=514)
target_compile_definitions(rp2040-dongle PUBLIC MAIN_BUFFER_SIZE=524)
target_compile_definitions(rp2040-dongle PUBLIC ENABLE_NETWORK_STATS)
target_compile_definitions(rp2040-dongle PUBLIC ENABLE_DYNAMIC_PAYLOADS)



target_include_directories(rp2040-dongle PRIVATE
	${CMAKE_CURRENT_LIST_DIR}
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
        RF24
        RF24Network
        RF24Mesh
        )

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(rp2040-dongle)

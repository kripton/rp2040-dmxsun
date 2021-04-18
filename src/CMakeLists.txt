cmake_minimum_required(VERSION 3.13)

## pico_sdk_import.cmake is copied from
## https://github.com/raspberrypi/pico-sdk/blob/master/external/pico_sdk_import.cmake
include(pico_sdk_import.cmake)

## Give the baby a name. Will also be the name of the resulting binaries
project(rp2040-dongle)

## Init the pico-sdk. Will instruct the user what to do if deps are missing
pico_sdk_init()

## Include the CMakeLists-files of (non-pico-sdk)-libraries we will be using
include(../lib/RF24/CMakeLists.txt)
include(../lib/RF24Network/CMakeLists.txt)
include(../lib/RF24Mesh/CMakeLists.txt)
include(../lib/heatshrink/CMakeLists.txt)

## Add our own C/C++ files here
add_executable(${CMAKE_PROJECT_NAME}
    dmahandler.c
    main.c
    hidinterface.c
    acminterface.c
    usb_descriptors.c
    log.cpp
    stdio_usb.c
    wireless.cpp
)

## Add our rp2040-PIO programs here
pico_generate_pio_header(${CMAKE_PROJECT_NAME}
    ${CMAKE_CURRENT_LIST_DIR}/tx16.pio
)

## Define the SPI pins we use for the RF24 library
target_compile_definitions(${CMAKE_PROJECT_NAME}
    PUBLIC PICO_DEFAULT_SPI_SCK_PIN=2
    PUBLIC PICO_DEFAULT_SPI_TX_PIN=3
    PUBLIC PICO_DEFAULT_SPI_RX_PIN=4
)

## Config for the RF24Network library
target_compile_definitions(${CMAKE_PROJECT_NAME}
    PUBLIC MAX_PAYLOAD_SIZE=514
    ENABLE_NETWORK_STATS
)

## Config for the RF24Mesh library
target_compile_definitions(${CMAKE_PROJECT_NAME}
    PUBLIC MESH_DEFAULT_CHANNEL=120
    PUBLIC MESH_RENEWAL_TIMEOUT=2000
    PUBLIC MESH_MEM_ALLOC_SIZE=16
    PUBLIC MESH_LOOKUP_TIMEOUT=250
    PUBLIC MESH_WRITE_TIMEOUT=100
)

## Config for the heatshrink library
target_compile_definitions(${CMAKE_PROJECT_NAME}
    PUBLIC HEATSHRINK_DYNAMIC_ALLOC=0
    PUBLIC HEATSHRINK_STATIC_INPUT_BUFFER_SIZE=600
    PUBLIC HEATSHRINK_STATIC_WINDOW_BITS=9
    PUBLIC HEATSHRINK_STATIC_LOOKAHEAD_BITS=6
)

## Needed so tusb_config.h is found by the tinyUSB-library included in the pico-sdk
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

## Pull in all pico-sdk + non-pico-sdk libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    pico_stdlib
    pico_unique_id
    pico_mem_ops
    tinyusb_device
    tinyusb_board
    hardware_dma
    hardware_i2c
    hardware_irq
    hardware_pio
    hardware_timer
    hardware_gpio
    hardware_spi
    RF24
    RF24Network
    RF24Mesh
    heatshrink
)

## Create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(${CMAKE_PROJECT_NAME})

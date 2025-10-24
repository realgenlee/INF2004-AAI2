# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
# == Keep your VS Code Pico extension preamble ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()

set(PICO_BOARD pico_w CACHE STRING "Board type")
cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(INF2004-AAI2 C CXX ASM)
pico_sdk_init()

add_executable(INF2004-AAI2
    src/main.c
    src/fsm.c
    src/drivers/motor.c
    src/drivers/encoder.c
    src/drivers/ir_line_follower.c
    src/drivers/ir_barcode_scanner.c
)

target_include_directories(INF2004-AAI2 PRIVATE include .)
target_link_libraries(INF2004-AAI2
    pico_stdlib
    hardware_pwm
    hardware_gpio
    hardware_irq
    hardware_timer
    hardware_adc
)

pico_enable_stdio_usb(INF2004-AAI2 1)
pico_enable_stdio_uart(INF2004-AAI2 0)
pico_add_extra_outputs(INF2004-AAI2)
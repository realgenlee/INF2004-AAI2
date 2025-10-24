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

# Set C/C++ standards before project
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

include(pico_sdk_import.cmake)

project(INF2004-AAI2 C CXX ASM)

# Always generate compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

pico_sdk_init()

add_executable(INF2004-AAI2
    src/main.c
    src/fsm.c
    src/drivers/motor.c
    src/drivers/encoder.c
    src/drivers/ir_line_follower.c
    src/drivers/ir_barcode_scanner.c
    src/drivers/ultrasonic.c
    src/drivers/magnetometer.c
)

target_include_directories(INF2004-AAI2 PRIVATE 
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(INF2004-AAI2
    pico_stdlib
    hardware_adc
    hardware_gpio
    hardware_i2c
    hardware_irq
    hardware_pwm
    hardware_timer
)

pico_enable_stdio_usb(INF2004-AAI2 1)
pico_enable_stdio_uart(INF2004-AAI2 0)
pico_add_extra_outputs(INF2004-AAI2)
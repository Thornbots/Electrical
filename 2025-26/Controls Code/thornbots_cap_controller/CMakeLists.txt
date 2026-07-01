# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(thornbots_cap_controller C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(thornbots_cap_controller
        thornbots_cap_controller.cpp
        buck_control.c
        calculations.c
)

pico_set_program_name(thornbots_cap_controller "thornbots_cap_controller")
pico_set_program_version(thornbots_cap_controller "0.1")

# COMMS_MODE 0 = USB Serial, 1 = UART
# Match this to #define COMMS_MODE in thornbots_cap_controller.cpp
pico_enable_stdio_uart(thornbots_cap_controller 0)
pico_enable_stdio_usb(thornbots_cap_controller 1)

# Include headers from project root
target_include_directories(thornbots_cap_controller PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(thornbots_cap_controller
        pico_stdlib
        pico_multicore
        pico_bootrom
        hardware_spi
        hardware_pwm
        hardware_dma
        hardware_gpio
        hardware_uart
        hardware_irq
)

pico_add_extra_outputs(thornbots_cap_controller)
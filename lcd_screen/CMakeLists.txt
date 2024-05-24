# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(lcd_screen C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
add_executable(lcd_screen


    # Main executalbes
    src/lcd_screen.cpp
)



pico_set_program_name(lcd_screen "lcd_screen")
pico_set_program_version(lcd_screen "0.1")

pico_enable_stdio_uart(lcd_screen 0)
pico_enable_stdio_usb(lcd_screen 1)

# Add any user requested libraries
add_subdirectory(pico-ssd1306)
target_link_libraries(lcd_screen
    pico_stdlib
    hardware_spi
    pico_ssd1306
    hardware_i2c
)




pico_add_extra_outputs(lcd_screen)


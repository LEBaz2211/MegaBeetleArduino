## CAN for Raspbery PICO
### SETUP
To begin you can install the [pico setup installer](https://github.com/raspberrypi/pico-setup-windows/releases/latest/download/pico-setup-windows-x64-standalone.exe) to be have the right environment(windows).

A more detailed explanation on the installation can be found in [the official document](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf).

In the project folder you will find a CmakeLists file that will define the libraries and sdk used for the projects<br><br>
Setup the project and add the libraries.

```
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
```
### UPLOAD FILE TO THE PICO CARD
In your project folder create a folder called **build**

In a terminal use the cd command to go into that folder

Use the command
```
cmake ../ -G "Unix Makefiles"
```
This will create files in the build folder. 
<br>
Still in the same terminal use the *make* command to create the file to upload to the pico.
```
make
```
<br>
This will create multiple additional files. The one you need is the file with *.uf2* extension.
<br>
Plug the pico in boot mode(by pressing on the button while pluging the usb) and upload the file.

### USAGE OF THE SCREEN

To use the lcd screen in the code a few main fonction are used:

-    lcd_clear()
-    lcd_set_cursor()
-    lcd_string()

lcd_clear() as the name implies, empties the screen of all text. Usefull to make sure no leftover text is left in a untreated box

lcd_set_cursor() takes two parameters, the line (0 or 1) and the column (0 to 15). Used for example if you want to keep the first line intact but update the second line or a specific character.

lcd_string() is the main function that will receive the text that you want to show on screen. Keep in mind the length is limited to 32 characters. Also keep in mind where the cursor is pointed at.
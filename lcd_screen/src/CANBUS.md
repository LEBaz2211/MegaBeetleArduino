# CAN BUS 

##  Message format

CANBUS communications uses a frame of data made of three parts
- An ID ex: 0x10
- Data Length Code(dlc) determines the length of the DATA ex: 8
- The data, an array of bytes.

## CAN for Raspbery PICO
### SETUP
To begin you can install the [pico setup installer](https://github.com/raspberrypi/pico-setup-windows/releases/latest/download/pico-setup-windows-x64-standalone.exe) to be have the right environment(windows)

AND the [CAN interface library](https://github.com/adamczykpiotr/pico-mcp2515)

In the project folder you will find a CmakeLists file that will define the libraries and sdk used for the projects<br><br>
Setup the project and add the libraries.

```
add_executable(pico-mcp2515
    # Library executables
    include/mcp2515/mcp2515.cpp

    # Main executalbes
    src/pico-mcp2515.cpp
)
# Add MCP2515 Lib
target_include_directories(pico-mcp2515 PUBLIC include/)

pico_set_program_name(pico-mcp2515 "pico-mcp2515")
pico_set_program_version(pico-mcp2515 "0.1")

pico_enable_stdio_uart(pico-mcp2515 0)
pico_enable_stdio_usb(pico-mcp2515 1)

# Add any user requested libraries
target_link_libraries(pico-mcp2515
    pico_stdlib
    hardware_spi
)

pico_enable_stdio_usb(pico-mcp2515 1) 
pico_enable_stdio_uart(pico-mcp2515 0) 


pico_add_extra_outputs(pico-mcp2515)
```
PICO_DEFAULT_SPI_SCK_PIN 18
PICO_DEFAULT_SPI_TX_PIN 19
PICO_DEFAULT_SPI_RX_PIN 16
PICO_DEFAULT_SPI_CSN_PIN 17


In the C++ file include the libraries
```
#include <stdio.h>
#include "pico/stdlib.h"
#include "mcp2515/mcp2515.h"
#include <stdint.h>

MCP2515 can0;
struct can_frame rx;
```

And in the main() function initialize the CAN
```
stdio_init_all();
// pico_enable_stdio_usb(NULL, 0);

//Initialize interface
can0.reset();
can0.setBitrate(CAN_1000KBPS, MCP_16MHZ);
can0.setNormalMode();
```
Be sure to change the BitRate parameters accordingly.


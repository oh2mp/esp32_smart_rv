# OH2MP ESP32 Smart RV

### An ESP32 based smart screen for a motorhome or caravan


The screen graphics use [Bodmer's TFT_eSPI library](https://github.com/Bodmer/TFT_eSPI).

The library needs to be configured for the right driver chip etc. and with this project it's enough
to change file libraries/TFT_eSPI/User_Setup.h lines 188-193 as follows:

`
#define TFT_MISO 19
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS   15  // Chip select control pin
#define TFT_DC    4  // Data Command control pin
#define TFT_RST   2  // Reset pin (could connect to RST pin)
`


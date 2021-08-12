#include <Adafruit_SPIFlash.h>

// On-board external flash (QSPI or SPI) macros should already
// defined in your board variant if supported
// - EXTERNAL_FLASH_USE_QSPI
// - EXTERNAL_FLASH_USE_CS/EXTERNAL_FLASH_USE_SPI
#if defined(EXTERNAL_FLASH_USE_QSPI)
    Adafruit_FlashTransport_QSPI flashTransport;
#elif defined(EXTERNAL_FLASH_USE_SPI)
    Adafruit_FlashTransport_SPI flashTransport(EXTERNAL_FLASH_USE_CS, EXTERNAL_FLASH_USE_SPI);
#else
#error No QSPI/SPI flash are defined on your board variant.h !
#endif

Adafruit_SPIFlash flash(&flashTransport);
Adafruit_M0_Express_CircuitPython pythonfs(flash);

void setup() {

}

void loop() {
    // Create or append to a data.txt file and add a new line
    // to the end of it.  CircuitPython code can later open and
    // see this file too!
    File data = pythonfs.open("data.txt", FILE_WRITE);
    if (data) {
        // Write a new line to the file:
        data.println("Hello CircuitPython from Arduino!");
        data.close();
        // See the other fatfs examples like fatfs_full_usage and fatfs_datalogging
        // for more examples of interacting with files.
        Serial.println("Wrote a new line to the end of data.txt!");
    }
    else {
        Serial.println("Error, failed to open data file for writing!");
    }
}
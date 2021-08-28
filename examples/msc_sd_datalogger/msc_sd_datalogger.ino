/*
  SD card datalogger

  This example shows how to log data from three analog sensors
  to an SD card using the SD library.

  The circuit:
   analog sensors on analog ins 0, 1, and 2
   SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4 (for MKRZero SD: SDCARD_SS_PIN)

  created  24 Nov 2010
  modified 9 Apr 2012
  by Tom Igoe

  This example code is in the public domain.

*/

#include <SPI.h>
#include <SD.h>
#include "Adafruit_TinyUSB.h"

const int chipSelect = A5;
Adafruit_USBD_MSC usb_msc;

Sd2Card card;
SdVolume volume;

void setup() {
    // Set disk vendor id, product id and revision with string up to 8, 16, 4 characters respectively
    usb_msc.setID("Adafruit", "SD Card", "1.0");

    // Set read write callback
    usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);

    // Still initialize MSC but tell usb stack that MSC is not ready to read/write
    // If we don't initialize, board will be enumerated as CDC only
    usb_msc.setUnitReady(false);
    usb_msc.begin();

    Serial.begin(115200);
    while ( !Serial ) delay(10);   // wait for native usb

    Serial.println("Adafruit TinyUSB Mass Storage SD Card example");

    Serial.print("\nInitializing SD card...");

    if ( !card.init(SPI_HALF_SPEED, chipSelect) )
    {
        Serial.println("initialization failed. Things to check:");
        Serial.println("* is a card inserted?");
        Serial.println("* is your wiring correct?");
        Serial.println("* did you change the chipSelect pin to match your shield or module?");
        while (1) delay(1);
    }

    // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
    if (!volume.init(card)) {
        Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
        while (1) delay(1);
    }
    Serial.println("done!");

    uint32_t block_count = volume.blocksPerCluster()*volume.clusterCount();

    Serial.print("Volume size (MB):  ");
    Serial.println((block_count/2) / 1024);
    // Set disk size, SD block size is always 512
    usb_msc.setCapacity(block_count, 512);

    usb_msc.setUnitReady(true); // Enable USB MSC read/write
}

void loop() {
    // if (millis() > 10000) { // After 10 seconds, allow USB port to open
    //     // usb_msc.begin();
    //     usb_msc.setUnitReady(true);
    // }
}

// ----- USB MSC CALLBACKS -----

// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and
// return number of copied bytes (must be multiple of block size)
int32_t msc_read_cb (uint32_t lba, void* buffer, uint32_t bufsize)
{
    (void) bufsize;
    return card.readBlock(lba, (uint8_t*) buffer) ? 512 : -1;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and 
// return number of written bytes (must be multiple of block size)
int32_t msc_write_cb (uint32_t lba, uint8_t* buffer, uint32_t bufsize)
{
    (void) bufsize;
    return card.writeBlock(lba, buffer) ? 512 : -1;
}

// Callback invoked when WRITE10 command is completed (status received and accepted by host).
// used to flush any pending cache.
void msc_flush_cb (void)
{
    // nothing to do
}
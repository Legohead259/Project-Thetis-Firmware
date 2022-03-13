/**
 * Simple MSC device with SD card
 * author: chegewara
 */
#include "sdusb.h"
#if CFG_TUD_MSC

#define SD_MISO  37
#define SD_MOSI  35
#define SD_SCK   36
#define SD_CS    34

SDCard2USB dev;

void setup() {
    Serial.begin(115200);
    pinMode(13, OUTPUT);

    if(dev.initSD(SD_SCK, SD_MISO, SD_MOSI, SD_CS)) {
        if(dev.begin()) {
            Serial.println("MSC lun 1 begin");
            digitalWrite(13, HIGH);
        } 
        else log_e("LUN 1 failed");
    } 
    else Serial.println("Failed to init SD");
}

void loop() {
  delay(1000);
}

#endif

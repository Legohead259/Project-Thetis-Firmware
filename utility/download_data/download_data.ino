/**
 * @file download_data.ino
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-25
 * 
 * @copyright Copyright (c) 2022
 * 
 * Downloads data from Thetis' on-board FFAT flash memory and prints the file contents
 * to the Serial port. Users can select which file to print and clear the memory when they are done
 */

// XTSD instantiation
// #include <SPI.h>
// #include <SD.h>
#include <FS.h>
#include <FFat.h>
#define NUM_FILES 32
#define FILENAME_LEN 12

// Neopixel instantiation
#include <Adafruit_NeoPixel.h>
#define NEO_EN_PIN 14
#define NEO_DATA_PIN 15
#define DASH_ON 500
#define DOT_ON 125
#define BLINK_INTERVAL 250
#define MESSAGE_INTERVAL 1000
#define MAXIMUM_BRIGHTNESS 32
#define NUM_STEPS 16
#define BRIGHTNESS_STEP MAXIMUM_BRIGHTNESS/NUM_STEPS

Adafruit_NeoPixel pixel(1, NEO_DATA_PIN, NEO_RGB + NEO_KHZ800);
const uint32_t OFF      =  pixel.Color(0, 0, 0);
const uint32_t WHITE    =  pixel.Color(255, 255, 255);
const uint32_t BLUE     =  pixel.Color(255, 0, 0);
const uint32_t RED      =  pixel.Color(0, 255, 0);
const uint32_t GREEN    =  pixel.Color(0, 255, 0);
const uint32_t PURPLE   =  pixel.Color(255, 0, 255);
const uint32_t AMBER    =  pixel.Color(255, 191, 0);
const uint32_t CYAN     =  pixel.Color(255, 255, 0);
const uint32_t LIME     =  pixel.Color(0, 255, 125);
const float brightness = 0.1;
uint8_t pixelState = 0;
bool brightnessInc = true;

char filename_arr[NUM_FILES][FILENAME_LEN]; // Create an array for filenames


void setup() {
    Serial.begin(115200);
    while(!Serial); // Wait for Serial connection

    Serial.println("---------------------------------------------");
    Serial.println("        Project Thetis Data Downloader       ");
    Serial.println("                 Version 0.1                 ");
    Serial.println("---------------------------------------------");
    Serial.println();

    Serial.print("Initializing file system...");
    if (!FFat.begin()) {
        Serial.print("FFat initialization failed");
        while (true); // TODO: Blink error code
    }
    Serial.println("done!");
    Serial.println();

    Serial.printf("Total space: %10u\n\r", FFat.totalBytes());
    Serial.printf("Free space: %10u\n\r", FFat.freeBytes());
    listDir(FFat, "/", 0);
    Serial.println();
    Serial.print("Please select a file to download or enter ""-1"" to clear drive: ");
}

void loop() {
    while(Serial.available()) {
        long c = Serial.parseInt();
        Serial.println(c);
        if(c) {
            char _fn[FILENAME_LEN];
            strcat(_fn, "/");
            strcat(_fn, filename_arr[c-1]);
            readFile(FFat, _fn);
        }
        else if(c == -1) {
            for(uint8_t i=0; i++; i<NUM_FILES) {
                char _fn[FILENAME_LEN];
                strcat(_fn, "/");
                strcat(_fn, filename_arr[i]);
                // deleteFile(FFat, "/"+filename_arr[i]);
                Serial.printf("Deleting file: %s", _fn);
            }
        }
        Serial.flush();
    }
    // TODO: Track down crashing bug!
}


// ================================
// ===== FILESYSTEM FUNCTIONS =====
// ================================


void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    uint8_t index = 0;

    Serial.printf("Listing directory: %s\r\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("- failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println(" - not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.path(), levels -1);
            }
        } else {
            strncpy(filename_arr[index], file.name(), FILENAME_LEN);
            index++;
            Serial.printf("  [%u] FILE: ", index);
            Serial.print(file.name());
            Serial.print("\tSIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\r\n", path);

    File file = fs.open(path);
    if(!file || file.isDirectory()){
        Serial.println("- failed to open file for reading");
        return;
    }

    Serial.println("- read from file:");
    while(file.available()){
        Serial.write(file.read());
    }
    // Serial.println("Finished reading!"); // DEBUG
    file.close();
    // Serial.println("Closed file"); // Debug
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\r\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("- failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("- file written");
    } else {
        Serial.println("- write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\r\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("- failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("- message appended");
    } else {
        Serial.println("- append failed");
    }
    file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
    Serial.printf("Renaming file %s to %s\r\n", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.println("- file renamed");
    } else {
        Serial.println("- rename failed");
    }
}

void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\r\n", path);
    if(fs.remove(path)){
        Serial.println("- file deleted");
    } else {
        Serial.println("- delete failed");
    }
}

void testFileIO(fs::FS &fs, const char * path){
    Serial.printf("Testing file I/O with %s\r\n", path);

    static uint8_t buf[512];
    size_t len = 0;
    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("- failed to open file for writing");
        return;
    }

    size_t i;
    Serial.print("- writing" );
    uint32_t start = millis();
    for(i=0; i<2048; i++){
        if ((i & 0x001F) == 0x001F){
          Serial.print(".");
        }
        file.write(buf, 512);
    }
    Serial.println("");
    uint32_t end = millis() - start;
    Serial.printf(" - %u bytes written in %u ms\r\n", 2048 * 512, end);
    file.close();

    file = fs.open(path);
    start = millis();
    end = start;
    i = 0;
    if(file && !file.isDirectory()){
        len = file.size();
        size_t flen = len;
        start = millis();
        Serial.print("- reading" );
        while(len){
            size_t toRead = len;
            if(toRead > 512){
                toRead = 512;
            }
            file.read(buf, toRead);
            if ((i++ & 0x001F) == 0x001F){
              Serial.print(".");
            }
            len -= toRead;
        }
        Serial.println("");
        end = millis() - start;
        Serial.printf("- %u bytes read in %u ms\r\n", flen, end);
        file.close();
    } else {
        Serial.println("- failed to open file for reading");
    }
}


// ========================
// ===NEOPIXEL FUNCTIONS===
// ========================


void initNeoPixel() {
    Serial.print("Initializing NeoPixel...");
    pinMode(NEO_EN_PIN, OUTPUT);
    digitalWrite(NEO_EN_PIN, LOW); // Enable NeoPixel
    pixel.begin(); // Initialize pins for output
    pixel.setBrightness(25);
    pixel.show();  // Turn all LEDs off ASAP
    Serial.println("done!");
}

void pulseLED(uint32_t color) {
    pixel.setBrightness(pixelState);
    pixel.setPixelColor(0, color);
    pixel.show();
    brightnessInc ? pixelState += BRIGHTNESS_STEP : pixelState -= BRIGHTNESS_STEP;
    if (pixelState >= MAXIMUM_BRIGHTNESS || pixelState <= 0) brightnessInc = !brightnessInc;
}

void rainbow(){
    pixel.setPixelColor(0, Wheel(&pixel,pixelState));
    pixel.show();
    pixelState++;
    if (pixelState >= 255) pixelState = 0;
}

uint32_t Wheel(Adafruit_NeoPixel *strip, byte WheelPos) {
    WheelPos = 255 - WheelPos;
    if(WheelPos < 85) {
        return strip->Color((255 - WheelPos * 3)*brightness, 0, WheelPos * 3*brightness);
    }
    if(WheelPos < 170) {
        WheelPos -= 85;
        return strip->Color(0, WheelPos * 3 * brightness, (255 - WheelPos * 3)*brightness);
    }
    WheelPos -= 170;
    return strip->Color(WheelPos * 3 * brightness, (255 - WheelPos * 3)*brightness, 0);
}

void blinkCode(byte code, uint32_t color) {
    bool dash = true;
    for (int n=0; n<4; n++) {
        if (bitRead(code, n)) {
            if (dash) {
                pixel.setPixelColor(0, color); pixel.show();
                delay(DASH_ON);
                pixel.setPixelColor(0, OFF); pixel.show();
                delay(BLINK_INTERVAL);
            }
            else {
                pixel.setPixelColor(0, color); pixel.show();
                delay(DOT_ON);
                pixel.setPixelColor(0, OFF); pixel.show();
                delay(BLINK_INTERVAL);
            }
        }
        else {
            if (dash) delay(DASH_ON+BLINK_INTERVAL);
            else delay(DOT_ON+BLINK_INTERVAL);
        }
        dash = !dash;
    }
    delay(MESSAGE_INTERVAL);
}
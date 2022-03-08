// Neopixel instantiation
#include <Adafruit_NeoPixel.h>
#define NEO_EN_PIN 14
#define NEO_DATA_PIN 15
#define DASH_ON 250
#define DOT_ON 125
#define BLINK_INTERVAL 125
#define MESSAGE_INTERVAL 1000
#define MAXIMUM_BRIGHTNESS 32
#define NUM_STEPS 16
#define BRIGHTNESS_STEP MAXIMUM_BRIGHTNESS/NUM_STEPS

/*
Code Table:
Error       |  DOT  |  DASH  |  DOT  |  DASH  |  CODE  |  COLOR  |  DESCRIPTION
------------|-------|--------|-------|--------|--------|---------|--------------------------------------------------
General     |   0   |   0    |   0   |    1   |  B0001 |   RED   | Unknown, but critical failure
XTSD Mount  |   0   |   0    |   1   |    0   |  B0010 |   RED   | XTSD filesystem fails to mount
Card Type   |   0   |   0    |   1   |    1   |  B0011 |   RED   | XTSD initializes, but reports a bad type
File Write  |   0   |   1    |   0   |    0   |  B0100 |   RED   | Datalog file fails to open
Radio       |   0   |   1    |   0   |    1   |  B0101 |   RED   | ESP32 radio fails to initialize/encounters error
GPS         |   0   |   1    |   1   |    0   |  B0110 |   RED   | GPS radio fails to initialize
IMU         |   0   |   1    |   1   |    1   |  B0111 |   RED   | IMU fails to initialize
Low Battery |   1   |   0    |   0   |    0   |  B1000 |  AMBER  | Battery voltage is below 3.0V

A dot is 125 ms on, 125 ms off
A dash is 250 ms on, 250 ms off
Space between codes is 1 sec
*/

enum ErrorCode {
    GEN_ERROR_CODE          = B0001,
    XTSD_MOUNT_ERROR_CODE   = B0010,
    CARD_TYPE_ERROR_CODE    = B0011,
    FILE_ERROR_CODE         = B0100,
    RADIO_ERROR_CODE        = B0101,
    GPS_ERROR_CODE          = B0110,
    IMU_ERROR_CODE          = B0111,
    LOW_BATT_ERROR_CODE     = B1000
};
 
/*
Status Table:
State           |  Color  |  Indication  |  Description
----------------|---------|--------------|---------------
Logging, No GPS |  BLUE   |    Solid     | Thetis is logging, but does not have a GPS fix
Logging, GPS    |  GREEN  |    Solid     | Thetis is logging with a GPS fix
Ready, No GPS   |  BLUE   |   Pulsing    | Accelerometer is calibrated but no GPS fix
Ready, GPS      |  GREEN  |   Pulsing    | Accelerometer is calibrated and there is a GPS fix
Standby         |  AMBER  |    Solid     | Accelerometer is not calibrated yet
*/

enum Status {
    LOGGING_NO_GPS,
    LOGGING_GPS,
    READY_NO_GPS,
    READY_GPS,
    STANDBY
};

Adafruit_NeoPixel pixel(1, NEO_DATA_PIN, NEO_BGR + NEO_KHZ800);
const uint32_t OFF      =  pixel.Color(0, 0, 0);       // BGR
const uint32_t WHITE    =  pixel.Color(255, 255, 255);
const uint32_t BLUE     =  pixel.Color(255, 0, 0);
const uint32_t RED      =  pixel.Color(0, 0, 255);
const uint32_t GREEN    =  pixel.Color(0, 255, 0);
const uint32_t PURPLE   =  pixel.Color(255, 0, 255);
const uint32_t AMBER    =  pixel.Color(0, 191, 255);
const uint32_t CYAN     =  pixel.Color(255, 255, 0);
const uint32_t LIME     =  pixel.Color(0, 255, 125);
uint8_t pixel_state = 0;
bool brightness_inc = true;
float brightness = 0.1;

void initNeoPixel() {
    Serial.print("Initializing NeoPixel...");
    pinMode(NEO_EN_PIN, OUTPUT);
    digitalWrite(NEO_EN_PIN, LOW); // Enable NeoPixel
    pixel.begin(); // Initialize pins for output
    pixel.setBrightness(50);
    pixel.show();  // Turn all LEDs off ASAP
    Serial.println("done!");
}

void setup() {
    initNeoPixel();
}

void loop() {
    // pulseLED(BLUE);
    delay(50); // Simulate doing other things
    rainbow();
}

void pulseLED(uint32_t color) {
    pixel.setBrightness(pixel_state);
    pixel.setPixelColor(0, color);
    pixel.show();
    brightness_inc ? pixel_state += BRIGHTNESS_STEP : pixel_state -= BRIGHTNESS_STEP;
    if (pixel_state >= MAXIMUM_BRIGHTNESS || pixel_state <= 0) brightness_inc = !brightness_inc;
}

void rainbow(){
    pixel.setPixelColor(0,Wheel(&pixel,pixel_state));
    pixel.show();
    pixel_state++;
    if(pixel_state>255) pixel_state = 0;
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
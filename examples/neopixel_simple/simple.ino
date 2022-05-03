#include <Adafruit_NeoPixel.h>

#define LED_COUNT 1
#define NEO_DATA_PIN 40
#define NEO_EN_PIN 39
#define DASH_ON 250
#define DOT_ON 125
#define BLINK_INTERVAL 125
#define MESSAGE_INTERVAL 1000

Adafruit_NeoPixel strip(LED_COUNT, LED_COUNT, NEO_GRB + NEO_KHZ800);
const uint32_t OFF      =  strip.Color(0, 0, 0);       // RGB
const uint32_t WHITE    =  strip.Color(255, 255, 255);
const uint32_t BLUE     =  strip.Color(0, 0, 255);
const uint32_t RED      =  strip.Color(255, 0, 0);
const uint32_t GREEN    =  strip.Color(0, 255, 0);
const uint32_t PURPLE   =  strip.Color(255, 0, 255);
const uint32_t AMBER    =  strip.Color(255, 191, 0);
const uint32_t CYAN     =  strip.Color(0, 255, 255);
const uint32_t LIME     =  strip.Color(125, 255, 0);

void setup() {
  pinMode(NEO_EN_PIN, OUTPUT);
  digitalWrite(NEO_EN_PIN, LOW); // Enable Neopixel

  strip.begin();
  strip.show();
  strip.setBrightness(10);
}


void loop() {
  // rainbow(10);
  blinkCode(B0101, CYAN);
}

void rainbow(int wait) {
  for(long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue += 256) {
    for(int i=0; i<strip.numPixels(); i++) { 
      int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
      strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
    }
    strip.show();
    delay(wait);
  }
}

void blinkCode(byte code, uint32_t color) {
    bool dash = true;
    for (int n=0; n<4; n++) {
        if (bitRead(code, n)) {
            if (dash) {
                strip.setPixelColor(0, color); strip.show();
                delay(DASH_ON);
                strip.setPixelColor(0, OFF); strip.show();
                delay(BLINK_INTERVAL);
            }
            else {
                strip.setPixelColor(0, color); strip.show();
                delay(DOT_ON);
                strip.setPixelColor(0, OFF); strip.show();
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
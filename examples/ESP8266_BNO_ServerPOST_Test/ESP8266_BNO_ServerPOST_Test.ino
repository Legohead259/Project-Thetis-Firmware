// Import required libraries
#include <ESP8266WiFi.h>
#include <ESPAsyncWebServer.h>

// Set your access point network credentials
const char* ssid = "ThetisNet";
const char* password = "123456789";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

void POSTDebug(uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        Serial.write(data[i]);
    }
    
    Serial.println();
}

void setTime(uint8_t *data, size_t len) {
    char buf[len];
    for (size_t i = 0; i < len; i++) { // Write request data to char buffer
        // Serial.write(data[i]); // Debug
        buf[i] = data[i];
    }
    // Serial.println(); // Debug
    char *str;
    char *p = buf;
    while ((str = strtok_r(p, ",", &p)) != NULL) { // delimiter is the semicolon
        Serial.println(str);  // DEBUG
    }
}

void setup() {
    // Serial port for debugging purposes
    Serial.begin(115200);
    Serial.println();
    
    // Setting the ESP as an access point
    Serial.print("Setting AP (Access Point)…");
    // Remove the password parameter, if you want the AP (Access Point) to be open
    WiFi.softAP(ssid);

    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);

    server.on(
    "/post",
    HTTP_POST,
    [](AsyncWebServerRequest * request){},
    NULL,
    [](AsyncWebServerRequest * request, uint8_t *data, size_t len, size_t index, size_t total) {
        POSTDebug(data, len);
        request->send(200);
    });

    server.on(
    "/settime",
    HTTP_POST,
    [](AsyncWebServerRequest * request){},
    NULL,
    [](AsyncWebServerRequest * request, uint8_t *data, size_t len, size_t index, size_t total) {
        setTime(data, len);
        request->send(200);
    });
    
    // Start server
    server.begin();
}
 
void loop() {
  
}
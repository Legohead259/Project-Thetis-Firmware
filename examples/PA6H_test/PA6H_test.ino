// General instantiation
#define ACT_LED_PIN 38

// GPS instantiation
#include <MicroNMEA.h>
#define GPS_PPS_PIN 42
// HardwareSerial GPS(0);
HardwareSerial& GPS = Serial1;
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));
bool ledState = LOW;
volatile bool ppsTriggered = false;
uint8_t lastSecond = 0;
float lastMillis = millis();
float curMillis = 0;
float curMSecond = 0;

void setup() {
    Serial.begin(115200);
    while(!Serial); // Wait for serial connection

    initGPS();
}

void loop() {
    testGPS();
}

// ===================
// ===GPS FUNCTIONS===
// ===================

void initGPS() {
    Serial.print("Initializing GPS..."); // DEBUG
    GPS.begin(9600); // Begin talking with GPS at 9600 baud
    if (!GPS) {
        Serial.println("Failed to initialize GPS"); // DEBUG
        while (true); // Block further code execution
    }
    nmea.setUnknownSentenceHandler(printUnknownSentence); // Set interrupt Routine for unrecognized sentences
    MicroNMEA::sendSentence(GPS, "$PORZB"); // Clear the list of messages which are sent
    MicroNMEA::sendSentence(GPS, "$PORZB,RMC,1,GGA,1"); // Send only RMC (minimum recommended data) and GGA (fix data) including altitude
    MicroNMEA::sendSentence(GPS, "$PNVGNME,2,9,1"); // Disable compatability mode (NV08C-CSM proprietary message) and adjust precision of time and position fields

    pinMode(GPS_PPS_PIN, INPUT);
    attachInterrupt(GPS_PPS_PIN, ppsHandler, RISING);

    Serial.println("done!"); // DEBUG
}

void testGPS() {
    long startTime = millis();
    if (ppsTriggered) { // NOTE: NMEA parsing will not begin until 3D fix is acheived. See PPS information in datasheet for details
        ppsTriggered = false;
        ledState = !ledState;
        digitalWrite(ACT_LED_PIN, ledState);

        // Output GPS information from previous second
        Serial.print("Valid fix: ");
        Serial.println(nmea.isValid() ? "yes" : "no");

        Serial.print("Nav. system: ");
        if (nmea.getNavSystem())
            Serial.println(nmea.getNavSystem());
        else
            Serial.println("none");

        Serial.print("Num. satellites: ");
        Serial.println(nmea.getNumSatellites());

        Serial.print("HDOP: ");
        Serial.println(nmea.getHDOP()/10., 1);

        Serial.print("Date/time: ");
        Serial.print(nmea.getYear());
        Serial.print('-');
        Serial.print(int(nmea.getMonth()));
        Serial.print('-');
        Serial.print(int(nmea.getDay()));
        Serial.print('T');
        Serial.print(int(nmea.getHour()));
        Serial.print(':');
        Serial.print(int(nmea.getMinute()));
        Serial.print(':');
        Serial.println(int(nmea.getSecond()));

        long latitude_mdeg = nmea.getLatitude();
        long longitude_mdeg = nmea.getLongitude();
        Serial.print("Latitude (deg): ");
        Serial.println(latitude_mdeg / 1000000., 6);

        Serial.print("Longitude (deg): ");
        Serial.println(longitude_mdeg / 1000000., 6);

        long alt;
        Serial.print("Altitude (m): ");
        if (nmea.getAltitude(alt))
            Serial.println(alt / 1000., 3);
        else
            Serial.println("not available");

        Serial.print("Speed: ");
        Serial.println(nmea.getSpeed() / 1000., 3);
        Serial.print("Course: ");
        Serial.println(nmea.getCourse() / 1000., 3);
    }
    while (!ppsTriggered && GPS.available()) { // Print out raw GPS data when PPS is not triggered (no 3D fix)
        char c = GPS.read();
        Serial.print(c);
        nmea.process(c);
    }
}

void ppsHandler(void) {
    ppsTriggered = true;
    // Serial.println("triggered!"); // DEBUG
}

void printUnknownSentence(MicroNMEA& nmea) {
    Serial.println();
	Serial.print("Unknown sentence: ");
	Serial.println(nmea.getSentence());
}
#define LOG_EN_PIN 41
#define LOG_PRESS_TIME 1000 // ms
#define ACT_LED_PIN 38

long logButtonStartTime = 0;
long logButtonPresses = 0;

void setup() {
    Serial.begin(115200);
    while(!Serial); // Wait for Serial connection

    pinMode(LOG_EN_PIN, INPUT);
    attachInterrupt(LOG_EN_PIN, logButtonPressISR, FALLING);

    pinMode(ACT_LED_PIN, OUTPUT);
}

void loop() {
    static bool _isLogging = false;
    static long _oldLogButtonPresses = logButtonPresses;

    if (logButtonPresses != _oldLogButtonPresses && !digitalRead(LOG_EN_PIN) && millis() >= logButtonStartTime+LOG_PRESS_TIME) { // Check if log button is pressed and has been held
        _isLogging = !_isLogging;
        _oldLogButtonPresses = logButtonPresses;
        Serial.printf("Logging is %s!\r\n", _isLogging ? "enabled" : "disabled");
    }
    digitalWrite(ACT_LED_PIN, _isLogging);
}

void logButtonPressISR() {
    logButtonPresses++;
    logButtonStartTime = millis();
    Serial.printf("Log Button Pressed: %d\n\r", logButtonPresses);
}
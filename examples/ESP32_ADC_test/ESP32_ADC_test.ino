#define BATT_MON_PIN 15

void setup() {
    Serial.begin(115200);
    while(!Serial); // wait for serial connection

    pinMode(BATT_MON_PIN, INPUT);

    analogSetAttenuation(ADC_0db);
    double rawADCVal = analogRead(BATT_MON_PIN);
    double voltagePerNum = 1.0/8192.0; // 1.1 = Vref
    double vBatMeasured = rawADCVal * voltagePerNum;
    double vBat = (vBatMeasured * (1E6 + 1E6)) / 1.0E6;

    Serial.printf("Method 1: %03f\t", vBat);
    Serial.printf("Method 2: %03f", (analogReadMilliVolts(BATT_MON_PIN) * (1E6 + 1E6)) / 1.0E6);
}

void loop() {

}
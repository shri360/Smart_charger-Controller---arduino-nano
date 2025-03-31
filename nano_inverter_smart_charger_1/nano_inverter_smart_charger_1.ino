#include <Wire.h>

// Pin Definitions
#define CHARGING_RELAY_PIN 3  // Relay for switching charging modes (Bulk <-> Float)
#define LOW_VOLTAGE_RELAY_PIN 7  // Relay for Low Voltage Cutoff
#define BUZZER_PIN 4        // Buzzer for EB status change
#define EB_STATUS_PIN 5     // Digital Input for EB detection (Potential-Free Contact)
#define CC_RELAY_PIN 6      // CC Relay (turns ON after 5 seconds)
#define VOLTAGE_SENSOR A0   // Voltage Sensor Input
#define CURRENT_SENSOR A1   // ACS712 Current Sensor Input

// Charging Conditions
const float FLOAT_VOLTAGE = 13.5;
const float BULK_VOLTAGE = 14.4;
const float BULK_VOLTAGE_THRESHOLD = 12.2;  // Switch back to Bulk mode
const float CHARGING_CURRENT_THRESHOLD = 1.0;

// Low voltage cutoff
const float LOW_VOLTAGE_CUTOFF = 10.5;
const float LOW_VOLTAGE_RECOVERY = 12.4;

bool lowVoltageState = false;  // For low voltage cutoff
unsigned long lowVoltageStartTime = 0;  // To track when voltage goes low
bool chargingMode = true;      // true = Bulk, false = Float
bool ebStatus = false;
unsigned long ebOnTime = 0;    // To track when EB came back
bool ccRelayState = false;

float readVoltage() {
    float sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += (analogRead(VOLTAGE_SENSOR) * 25.0) / 1023.0;
        delay(5);
    }
    return (sum / 10.0) * 0.956; // Averaged + Corrected
}

float readCurrent() {
    float sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += ((analogRead(CURRENT_SENSOR) - 512) * 30.0) / 512.0;
        delay(5);
    }
    return (sum / 10.0) * 0.850; // Averaged + Corrected
}

void checkChargingMode(float voltage, float current) {
    if (chargingMode && current < CHARGING_CURRENT_THRESHOLD) {
        // Switch to Float Mode when current is low
        digitalWrite(CHARGING_RELAY_PIN, LOW);
        chargingMode = false;
        Serial.println("Charging Mode: Float");
    } else if (!chargingMode && voltage < BULK_VOLTAGE_THRESHOLD) {
        // Switch back to Bulk Mode when voltage drops
        digitalWrite(CHARGING_RELAY_PIN, HIGH);
        chargingMode = true;
        Serial.println("Charging Mode: Bulk");
    }
}

void checkLowVoltageCutoff(float voltage) {
    if (voltage <= LOW_VOLTAGE_CUTOFF) {
        if (lowVoltageStartTime == 0) {
            lowVoltageStartTime = millis();  // Start the timer
        }
        if (millis() - lowVoltageStartTime >= 2500 && !lowVoltageState) { // 2.5-second delay
            digitalWrite(LOW_VOLTAGE_RELAY_PIN, LOW); // Cut off load
            lowVoltageState = true;
            Serial.println("Low Voltage Cutoff: Activated");
        }
    } else {
        lowVoltageStartTime = 0; // Reset the timer
        if (voltage >= LOW_VOLTAGE_RECOVERY && lowVoltageState) {
            digitalWrite(LOW_VOLTAGE_RELAY_PIN, HIGH); // Restore power
            lowVoltageState = false;
            Serial.println("Low Voltage Cutoff: Deactivated");
        }
    }
}

void checkEBStatus() {
    bool currentEBStatus = !digitalRead(EB_STATUS_PIN); // Since NC = EB ON, we invert the reading

    if (currentEBStatus != ebStatus) {
        ebStatus = currentEBStatus;
        digitalWrite(BUZZER_PIN, HIGH);
        delay(500);
        digitalWrite(BUZZER_PIN, LOW);

        Serial.print("EB Status: ");
        Serial.println(ebStatus ? "Available" : "Not Available");

        if (ebStatus) {
            ebOnTime = millis();  // Start delay timer
            ccRelayState = false; // Reset CC relay state
        } else {
            digitalWrite(CC_RELAY_PIN, LOW); // Turn off CC relay when EB goes
        }
    }
}

void checkCCRelay() {
    if (ebStatus && !ccRelayState) {
        if (millis() - ebOnTime >= 5000) { // 5-second delay
            digitalWrite(CC_RELAY_PIN, HIGH);
            ccRelayState = true;
            Serial.println("CC Relay Activated after 5 seconds");
        }
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(CHARGING_RELAY_PIN, OUTPUT);
    pinMode(LOW_VOLTAGE_RELAY_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(CC_RELAY_PIN, OUTPUT);
    pinMode(EB_STATUS_PIN, INPUT_PULLUP); // Use internal pull-up

    pinMode(VOLTAGE_SENSOR, INPUT);
    pinMode(CURRENT_SENSOR, INPUT);
    
    digitalWrite(CHARGING_RELAY_PIN, HIGH); // Start in Bulk Mode
    digitalWrite(LOW_VOLTAGE_RELAY_PIN, HIGH); // Ensure Load is connected
    digitalWrite(CC_RELAY_PIN, LOW); // Ensure CC relay is off initially
}

void loop() {
    float batteryVoltage = readVoltage();
    float chargingCurrent = readCurrent();
    
    Serial.print("Voltage: ");
    Serial.print(batteryVoltage);
    Serial.print(" V, Current: ");
    Serial.print(chargingCurrent);
    Serial.print(" A, EB Status: ");
    Serial.print(ebStatus ? "Available" : "Not Available");

    Serial.print(", Charging Mode: ");
    Serial.print(chargingMode ? "Bulk" : "Float");

    Serial.print(", Low Voltage Cutoff: ");
    Serial.println(lowVoltageState ? "Activated" : "Deactivated");

    checkChargingMode(batteryVoltage, chargingCurrent);
    checkLowVoltageCutoff(batteryVoltage);
    checkEBStatus();
    checkCCRelay();
    
    delay(1000);
}

#include <Arduino.h>
// NVM library
#include <Preferences.h>
// Current sensors library
#include <Adafruit_INA219.h>
#include <Wire.h>
#define DEBUG

Preferences nvmlib;
Adafruit_INA219 ina219_1(0x44); // Address for the first module
Adafruit_INA219 ina219_2(0x40); // Address for the second module

int noiseCount1 = 0; // Noise count for sensor 1
int noiseCount2 = 0; // Noise count for sensor 2

float CurrentActl1, CurrentLast1; // Current values for sensor 1
float CurrentActl2, CurrentLast2; // Current values for sensor 2

int usersCount1 = 0; // Users count for sensor 1
int usersCount2 = 0; // Users count for sensor 2

int deleteCounter = 0;

void setup() {
    // ---------- Current sensors ----------------
    Serial.begin(115200);
    Wire.begin(); // Initialize I2C communication

    if (!ina219_1.begin()) {
        Serial.println("Failed to find INA219 at 0x44");
        while (1);
    }
    Serial.println("INA219 (0x44) initialized");

    if (!ina219_2.begin()) {
        Serial.println("Failed to find INA219 at 0x40");
        while (1);
    }
    Serial.println("INA219 (0x40) initialized");

    CurrentActl1 = abs(ina219_1.getCurrent_mA());
    CurrentLast1 = CurrentActl1;

    CurrentActl2 = abs(ina219_2.getCurrent_mA());
    CurrentLast2 = CurrentActl2;

    // ----------- NVM -----------------------
    nvmlib.begin("storage", false); // Namespace "storage" in read/write mode

    // Restore counters from NVM
    usersCount1 = nvmlib.getInt("usersCount1", 0); // Default value is 0 if not set
    usersCount2 = nvmlib.getInt("usersCount2", 0);
    Serial.println("Restored counters:");
    Serial.println("Sensor 1: " + String(usersCount1));
    Serial.println("Sensor 2: " + String(usersCount2));
}

void loop() {
    // Check for connections and updates for sensor 1
    if (SomebodyConnected(ina219_1, CurrentLast1)) {
        noiseCount1++;
        if (noiseCount1 == 3) {
            usersCount1++;
            nvmlib.putInt("usersCount1", usersCount1); // Update NVM
            CurrentLast1 = CurrentActl1;
        }
    } else if (SomebodyDisconnected(ina219_1, CurrentLast1)) {
        noiseCount1++;
        if (noiseCount1 == 3) {
            CurrentLast1 = CurrentActl1;
        }
    } else {
        noiseCount1 = 0;
    }

    // Check for connections and updates for sensor 2
    if (SomebodyConnected(ina219_2, CurrentLast2)) {
        noiseCount2++;
        if (noiseCount2 == 3) {
            usersCount2++;
            nvmlib.putInt("usersCount2", usersCount2); // Update NVM
            CurrentLast2 = CurrentActl2;
        }
    } else if (SomebodyDisconnected(ina219_2, CurrentLast2)) {
        noiseCount2++;
        if (noiseCount2 == 3) {
            CurrentLast2 = CurrentActl2;
        }
    } else {
        noiseCount2 = 0;
    }

    // Serial commands
    if (Serial.available() > 0) {
        char input = Serial.read();
        if (input == 'r') {
            Serial.println("Sensor 1: " + String(usersCount1));
            Serial.println("Sensor 2: " + String(usersCount2));
            deleteCounter = 0;
        } else if (input == 'd') {
            deleteCounter++;
            Serial.println("Do you want to delete counters? Press 'd' to confirm or 'r' to cancel");
            if (deleteCounter == 2) {
                deleteCounter = 0;
                usersCount1 = 0;
                usersCount2 = 0;
                nvmlib.putInt("usersCount1", usersCount1);
                nvmlib.putInt("usersCount2", usersCount2);
                Serial.println("Counters reset");
            }
        }
    }

    CurrentActl1 = abs(ina219_1.getCurrent_mA());
    CurrentActl2 = abs(ina219_2.getCurrent_mA());
    #ifdef DEBUG
        Serial.print("DEBUG:current value 1 ");
        Serial.println(CurrentActl1);
        Serial.print("DEBUG:current value 2 ");
        Serial.println(CurrentActl2);
    #endif

    delay(500);
}

// Functions to detect connection/disconnection for each sensor
bool SomebodyConnected(Adafruit_INA219 &sensor, float &currentLast) {
    float currentActl = abs(sensor.getCurrent_mA());
    if ((currentActl - 450) >= currentLast) {
        return true;
    } else {
        return false;
    }
}

bool SomebodyDisconnected(Adafruit_INA219 &sensor, float &currentLast) {
    float currentActl = abs(sensor.getCurrent_mA());
    if ((currentActl + 450) <= currentLast) {
        return true;
    } else {
        return false;
    }
}
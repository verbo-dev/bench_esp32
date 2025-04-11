#include <Arduino.h>
//nvm library
#include <Preferences.h>
//current sensors library
#include <Adafruit_INA219.h>
#include <Wire.h>

Preferences nvmlib;
Adafruit_INA219 ina219_1(0x44); // Address for the first module
Adafruit_INA219 ina219_2(0x40); // Address for the second module

int noiseCount = 0;

float CurrentActl;
float CurrentLast;

int usersCount = 0;

float getCurrent();
bool SomebodyDisconnected();
bool SomebodyConnected();

int deleteCounter = 0;
void setup()
{
    // ---------- current sensors ----------------
    Serial.begin(115200);
    Wire.begin(); // Initialize I2C communication

    if (!ina219_1.begin()) {
        Serial.println("Failed to find INA219 at 0x44");
        while (1);
    }
    Serial.println("INA219 (0x44) initialized");

    // Initialize the second INA219 module
    if (!ina219_2.begin()) {
        Serial.println("Failed to find INA219 at 0x40");
        while (1);
    }
    Serial.println("INA219 (0x40) initialized");

    CurrentActl = getCurrent();
    CurrentLast = CurrentActl;

    //------------ nvm -----------------------

    nvmlib.begin("storage", false);  // Namespace "storage" in read/write mode

    // Retrieve the last saved value from NVM
    usersCount = nvmlib.getInt("usersCount", 0); // Default value is 0 if not set
    Serial.println("Restored counter value: " + String(usersCount));
}

void loop()
{

    if (SomebodyConnected() == true)
    {
        noiseCount++;
        if(noiseCount == 3) //if we have 3 times the same reading then it is not noise
        {
            usersCount++; //write in NVM
            nvmlib.putInt("usersCount", usersCount);
            CurrentLast = CurrentActl;
        }
    }
    else if (SomebodyDisconnected() == true)//someone just disconnected
    {
        noiseCount++;
        if(noiseCount == 3) //if we have 3 times the same reading then it is not noise
        {
            CurrentLast = CurrentActl;
        }
    }

    else 
    {
        noiseCount = 0;
    }


    if (Serial.available() > 0) {
        char input = Serial.read(); // Read and store the input once
        
        if (input == 'r') {
            Serial.println(usersCount);
            deleteCounter = 0;
        } else if (input == 'd') {
            deleteCounter++;
            Serial.println("Do you want to delete the counter? Press 'd' key to confirm or 'r' to cancel");
            if (deleteCounter == 2)
            {
              deleteCounter = 0;
              usersCount = 0;
              nvmlib.putInt("usersCount", usersCount);
              Serial.println("reset");
            }
        }
    }
    CurrentActl = getCurrent();

    delay(500);
}

bool SomebodyConnected()
{

    if((CurrentActl - 500) >= CurrentLast)
    return true;

    else
    return false;
}

bool SomebodyDisconnected()
{
    if((CurrentActl + 500) <= CurrentLast)
    return true;

    else
    return false;
}

float getCurrent()
{
    return ina219_1.getCurrent_mA() + ina219_2.getCurrent_mA();
}
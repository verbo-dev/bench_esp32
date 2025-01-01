#include <Arduino.h>
float floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
//sensor voltaje:::::::::::::
#define ANALOG_PIN1  36 // ESP32 pin GPIO36 (ADC0) connected to voltage sensor
#define REF_VOLTAGE    3.3
#define ADC_RESOLUTION 4096.0
#define R1             30000.0 // resistor values in voltage sensor (in ohms)
#define R2               7500.0  // resistor values in voltage sensor (in ohms)

//DATOS DE LA BANCA
#define num_dispositivo 1

//::::::::::::::::::::::
#if defined(ESP32)
  #include <WiFi.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
#endif
#include <Firebase_ESP_Client.h>

//Provide the token generation process info.
#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert your network credentials
#define WIFI_SSID "IZZI-34B0"
#define WIFI_PASSWORD "9CC8FC7734B0"

// Insert Firebase project API Key
#define API_KEY ""


// Insert RTDB URLefine the RTDB URL */
#define DATABASE_URL "https://bench-verbo-default-rtdb.firebaseio.com/"

//Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;
int count = 0;
bool signupOK = false;

float voltage = 0.0;
#define LED_PIN 18
bool CargadorStatus = false;
void setup() {
  //SENSOR VOLTAJE::: 
   // set the ADC attenuation to 11 dB (up to ~3.3V input)
  analogSetAttenuation(ADC_11db);
  //:::::::::::::::::::::::::::

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  /* Assign the api key (required) */
  config.api_key = API_KEY;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Sign up */
  if (Firebase.signUp(&config, &auth, "", "")){
    Serial.println("ok");
    signupOK = true;
  }
  else{
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h
  
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  // initialize digital pin GPIO18 as an output.
  pinMode(18, OUTPUT);
  Serial.begin(115200);
  // set the ADC attenuation to 11 dB (up to ~3.3V input)
  analogSetAttenuation(ADC_11db);
}


// the loop function runs over and over again forever
void loop() {
  
  // :::::::::::SENSOR VOLTAJE
  //read the analog input
  int adc_value = analogRead(ANALOG_PIN1);

  // determine voltage at adc input
  float voltage_adc = ((float)adc_value * REF_VOLTAGE) / ADC_RESOLUTION;

  // calculate voltage at the sensor input
  float voltage_in1 = voltage_adc * (R1 + R2) / R2;

  // print results to serial monitor to 2 decimal places
  //Serial.print("Measured Voltage = ");
  //Serial.println(voltage_in, 2);
//::::::::::::::::::::::::
  if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 500 || sendDataPrevMillis == 0)){
    sendDataPrevMillis = millis();
    // Write an Int number on the database path test/int
    
    int analogValue = analogRead(36);
    // Rescale to potentiometer's voltage (from 0V to 3.3V):
    voltage = floatMap(analogValue, 0, 4095, 0, 3.3);

    if (Firebase.RTDB.setInt(&fbdo, "Sensor/voltage", voltage_in1)){
      Serial.println(); Serial.print(voltage_in1);
      Serial.print(" - succesfully saved to: " + fbdo.dataPath());
      Serial.println(" (" + fbdo.dataType()+ ") ");
    }
    else {
      Serial.println("FAILED" + fbdo.errorReason());
    }
    
    
    
  }

  if(Firebase.RTDB.getBool(&fbdo,"/Cargador/activado")){
    if(fbdo.dataType()== "boolean") {
      CargadorStatus = fbdo.boolData();
      Serial.println("Successful READ from " + fbdo.dataPath() + ": " + CargadorStatus + " (" + fbdo.dataType() + ")");
      digitalWrite(LED_PIN, CargadorStatus);
    }

  }else{
    Serial.println("FAILED: " + fbdo.errorReason());
  }


  //digitalWrite(18, HIGH); // turn the LED on
  
  
  // Serial.print("led encendido, ");
  //Serial.println(voltage);
  //delay(500);             // wait for 500 milliseconds
  //digitalWrite(18, LOW);  // turn the LED off
  //Serial.println("led apagado,");
  //delay(500);             // wait for 500 milliseconds
}

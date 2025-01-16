#include <Arduino.h>

#include <WiFi.h>
//firebase libraries
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
//big query (MQTT connection) libraries
#include <PubSubClient.h>
//time calculation libraries 
#include <NTPClient.h>
#include <TimeLib.h>
//to interface the current sensor
#include <Adafruit_INA219.h>

// --- wi fi connection ----
#define WIFI_SSID "INFINITUMEDA2_2.4"
#define WIFI_PASSWORD "5JRCfA5n5Z"
WiFiClient esp32Client; //object for MQTT
bool wifi_begin(void);
bool NO_WIFI_MODE = false;


// ---- firebase connection -----
// Insert Firebase project API Key
#define API_KEY "AIzaSyC19dmmmmWnSyDrSgTtjwdyMqbRHhJzrhg"
// Insert RTDB URLefine the RTDB URL */
#define DATABASE_URL "https://bench-verbo-default-rtdb.firebaseio.com/"
//Define Firebase Data object
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
void firebase_config(void);
//token generation callback function
void tokenStatusCallback(String); 


// ---- Big Query (MQTT connection) -----------
//mqtt server connection
PubSubClient mqttClient(esp32Client);
#define server "34.66.172.243" //remote google computer server
#define port 1883
const char * la_caldera_logger_t = "la_caldera/Bench1";
void mqtt_connect(void);
void callback(char*, byte*, unsigned int);

//for callback function
int var;
String resultS = "";

// ----- poll submits monitoring and charge control variables -----
bool signupOK = false;
bool get_cargador_status(void); //read from firebase 
bool CargadorStatus = false;
bool monitor_connection(void);
void write_firebase(bool);
bool user_charging = false;
//current sensor object
Adafruit_INA219 ina219;

// ----- general timers -----------
#define NO_WIFI_MODE_ACTIVATION_t 83 // NO_WIFI_MODE_ACTIVATION = NO_WIFI_MODE_ACTIVATION_t * internal function delay
#define BIGQUERY_PUBLISHING_t 720 //BIGQUERY_PUBLISHING = BIGQUERY_PUBLISHING_t (in seconds) / global void loop delay (in seconds)
#define USER_NOT_CHARGING_TIMEOUT_t 40 //USER_NOT_CHARGING_TIMEOUT = USER_NOT_CHARGING_TIMEOUT_t * global void loop delay
#define GLOBAL_VOID_LOOP_DELAY 500 //ms 


// ------ voltage and sensor processing variables -------------
float voltage_panel = 0.0;
float voltage_battery = 0.0;
float voltage_battery_filtered = 0.0;
float voltage_panel_filtered = 0.0;
float filtering(float);
#define RELAY_PIN 5
//sensor voltaje:::::::::::::
#define ANALOG_PIN_PANEL  36 // ESP32 pin GPIO36 (ADC0) connected to voltage sensor
#define ANALOG_PIN_BATTERY 12
#define VOLTAGE_AVG_SIZE 5
#define REF_VOLTAGE    3.3
#define ADC_RESOLUTION 4096.0
#define R1_SENSOR             30000.0 // resistor values in voltage sensor (in ohms)
#define R2_SENSOR               7500.0  // resistor values in voltage sensor (in ohms)
#define R1_PCB                1000.0 //resistor value in extra voltage pcb divider
#define R2_PCB                2000.0 

float voltage_measuring(uint8_t);
float current_calculation(void);
#define NO_USERS_CHARGING_THRESHOLDVAL 400.0
uint8_t user_not_charging_timeout = USER_NOT_CHARGING_TIMEOUT_t;
bool restart_voltage_filtering = false;
float vpanel_avg[VOLTAGE_AVG_SIZE];
float vbattery_avg[VOLTAGE_AVG_SIZE];
uint8_t vmeasurements_counter = 0;

// ----- sensor data publishing variables -------------
#define clave_disp 01
// - date related variables - 
//server works with UDP protocol
WiFiUDP ntpUDP;
//page of the server 
NTPClient timeClient(ntpUDP, "pool.ntp.org");
String date_calculation(void);
String time_calculation(void);
String folio_calculation(void);
String usuario_cargando(void);

//folio calculation
int current_day;
int last_day = current_day;
uint16_t folio_accumulation = 0;

int bigquery_publishing_timeout = BIGQUERY_PUBLISHING_t;


void setup() {
  //SENSOR VOLTAJE::: 
   // set the ADC attenuation to 11 dB (up to ~3.3V input)
  analogSetAttenuation(ADC_11db);
  //:::::::::::::::::::::::::::

  // -------------------------------- CONNECTION TO WIFI ---------------------------------------------

  NO_WIFI_MODE = wifi_begin();

  // ------------------------------ CONNECTION TO FIREBASE ----------------------------------------

  if(NO_WIFI_MODE == false)
  {
    firebase_config();
    Firebase.begin(&config, &auth);
  }
  
  // ------------------------------CONNECTION TO BIG QUERY (MQTT)------------------------------------
  if(NO_WIFI_MODE == false)
  {
    mqtt_connect();
  }

  // initialize digital pin GPIO18 as an output.
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(ANALOG_PIN_BATTERY, INPUT);
  pinMode(ANALOG_PIN_PANEL, INPUT);
  Serial.begin(115200);

  //for sensor data publishing to calculate date and time
  timeClient.begin();
  timeClient.setTimeOffset(-21600); //para llegar a una zona horaria GMT-6

  //I2C communication with current sensor begins
  ina219.begin(); //pins 21 and 22 of ESP32 as default

}


// the loop function runs over and over again forever
void loop() {


  // ------ POLL SUBMITS MONITORING AND CHARGE CONTROL ------------
  if ((Firebase.ready() && signupOK) && (NO_WIFI_MODE == false)) //if everything is ok in firebase
  {
    if(CargadorStatus == true)
    {
      CargadorStatus = monitor_connection(); //check if the user is connected to the chargers using current sensor
      if(CargadorStatus == false)
      {
        write_firebase(false); 
        digitalWrite(RELAY_PIN, LOW); //turn off the chargers
      }
      
    }
    else
    {
      CargadorStatus = get_cargador_status(); //poll if the status changed from false
      if(CargadorStatus == true)
      {
        user_charging = true; //for sensor data publishing to know that in this BIGQUERY_PUBLISHING time lapse somebody was connected
        digitalWrite(RELAY_PIN, HIGH); //turn on the chargers
      }
    }   
  }
  else if (NO_WIFI_MODE == true) //chargers will be always working if there is no internet
  {
    digitalWrite(RELAY_PIN, HIGH); 
  }

  // --------------- VOLTAGE AND SENSOR PROCESSING ----------------------------

  voltage_battery = voltage_measuring(ANALOG_PIN_BATTERY);
  //voltage_battery_filtered = filtering(voltage_battery);

  voltage_panel = voltage_measuring(ANALOG_PIN_PANEL);
  //voltage_panel_filtered = filtering(voltage_panel);

    // --------------- WIFI TROUBLESHOOTING --------------------------------------
  if(WiFi.status() != WL_CONNECTED) //check if the connection to internet is still stable 
  {
    NO_WIFI_MODE = wifi_begin();
    if(NO_WIFI_MODE == false) //reconnect to everything if there is wifi connection again
    {
      Firebase.begin(&config, &auth);
      mqtt_connect(); //this function will BLOCK the program if connection with google remote computer is not succesfull
    }
  }

  // --------------- SENSOR DATA PUBLISHING --------------------------------------

  bigquery_publishing_timeout = bigquery_publishing_timeout - 1;

  if((bigquery_publishing_timeout == 0) && (NO_WIFI_MODE == false))
  {
    bigquery_publishing_timeout = BIGQUERY_PUBLISHING_t;

    //prepare variables to be published on mqtt
    String voltage_panel_filteredS = String(voltage_panel);
    String voltage_battery_filteredS = String(voltage_battery);

    String DatatoPublish = voltage_panel_filteredS + "|" + voltage_battery_filteredS + "|" + clave_disp \
                           + "|" + date_calculation() + "|" + time_calculation() + "|" + folio_calculation() \
                           + "|" + usuario_cargando();

    //publish in mqtt 
    mqttClient.publish(la_caldera_logger_t,DatatoPublish.c_str(),false); //retain set to false
    Serial.println("publishing this data:" + DatatoPublish + "into topic:");
    Serial.print(la_caldera_logger_t);
    restart_voltage_filtering = true; //data will be filtered and averaged again after each publication
  }
  else if (NO_WIFI_MODE == true)
  {
    //logic to save the sensor data in a separate variable 
  }

  delay(GLOBAL_VOID_LOOP_DELAY);
}

// ------------------ ALL CONNECTION FUNCTIONS -------------------------------

bool wifi_begin(void)
{
  uint8_t wifi_timeout = NO_WIFI_MODE_ACTIVATION_t; // NO_WIFI_MODE_ACTIVATION = wifi_timeout * delay
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while ((WiFi.status() != WL_CONNECTED) && (wifi_timeout != 0)){
    Serial.print(".");
    delay(300);
    wifi_timeout = wifi_timeout - 1;
  }

  if(wifi_timeout == 0) //if after x seconds the wifi is not connected
  {
    Serial.println();
    Serial.print("WIFI connection ERROR");
    return true; //NO_WIFI_MODE = true
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  return false;

}

void firebase_config(void)
{
    /* Assign the api key (required) */
  config.api_key = API_KEY;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Sign up */
  if (Firebase.signUp(&config, &auth, "", "")){
    Serial.println("Firebase signup successfull");
    signupOK = true;
  }
  else{
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h
}

//token generation callback function
void tokenStatusCallback(String result) 
{ 
    Serial.println("Token status: " + result); 
}

void mqtt_connect() 
{
  mqttClient.setServer(server, port);
  mqttClient.setKeepAlive((BIGQUERY_PUBLISHING_t / 2)+10); // +10 just to have a gap for the keepalive
  mqttClient.setCallback(callback);
  while (!mqttClient.connected()) {
    Serial.print("MQTT connection in progress...");
    String client = String(clave_disp);
    if (mqttClient.connect(client.c_str())) { /*this name is assigned in order 
                                                                        for the MQTT server (inside google
                                                                        cloud remote computer) to recognize this device */
      Serial.println("MQTT connection successfull");

      mqttClient.subscribe(la_caldera_logger_t);

      
    } else {
      Serial.print("MQTT connection failed, response code=");
      Serial.print(mqttClient.state());
      Serial.println("trying again...");
      // Wait 2 seconds before retrying
      delay(2000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message recieved from mqtt server [");
  Serial.print(topic);
  Serial.print("] ");

  char payload_string[length + 1];
  
  int resultI;

  memcpy(payload_string, payload, length);
  payload_string[length] = '\0';
  resultI = atoi(payload_string);
  
  var = resultI;

  resultS = "";
  
  /*for (int i=0;i<length;i++) {
    resultS= resultS + (char)payload[i];
  }
  Serial.println(resultS);*/
}

// ------------------------- SENSOR PROCESSING AND POLL SUBMITS MONITORING FUNCTIONS ----------------------

float voltage_measuring(uint8_t analog_pin)
{
  //read the analog input
  int adc_value = analogRead(analog_pin);
  // determine voltage at adc input
  float voltage_adc = ((float)adc_value * REF_VOLTAGE) / ADC_RESOLUTION;
  // calculate voltage at the sensor input
  float voltage = (voltage_adc * (R1_SENSOR + R2_SENSOR)) / R2_SENSOR;

  if(analog_pin == ANALOG_PIN_BATTERY)
  {
    return voltage;
  }
  //Voltage measuring from panel has an extra voltage divider
  return (voltage * (R1_PCB + R2_PCB)) / R2_PCB;

}

bool get_cargador_status(void)
{
  bool CargadorStatus = false;
  if(Firebase.RTDB.getBool(&fbdo,"/Cargador/activado"))
  {
    if(fbdo.dataType()== "boolean")
    {
      CargadorStatus = fbdo.boolData();
      Serial.println("Successful READ from " + fbdo.dataPath() + ": " + CargadorStatus + " (" + fbdo.dataType() + ")");
      return CargadorStatus;
    }
  
    else
    {
      Serial.println("FAILED: " + fbdo.errorReason());
      CargadorStatus = false; 
      return CargadorStatus;
    }
  } 
}

bool monitor_connection(void)
{
  float current_value = current_calculation();

  if(current_value <= NO_USERS_CHARGING_THRESHOLDVAL) //when there are no cellphones connected to the chargers
  {
    user_not_charging_timeout = user_not_charging_timeout - 1;
    if(user_not_charging_timeout == 0)
    {
      return false; //no user is longer connected to the ports
    } 
  }
  else
  {
    user_not_charging_timeout = USER_NOT_CHARGING_TIMEOUT_t;
  }
  return true; //a user is still connected to the chargers 

}

float current_calculation(void)
{
  return ina219.getCurrent_mA();
}

void write_firebase(bool state)
{
  if (Firebase.RTDB.setInt(&fbdo, "/Cargador/activado", state))
  {
    Serial.print(" - succesfully saved to: " + fbdo.dataPath());
    Serial.println(" (" + fbdo.dataType()+ ") ");
  }
  else 
  {
    Serial.println("FAILED" + fbdo.errorReason());
  }
}

/*In order to send correct data to bigquery the technique to filter the trash data is:
take 5 readings, do an average and discard the value that is very different from the average*/

/*
float filtering (float vvalue)
{
  float voltage_accumulator = 0;
  if(vmeasurements_counter <= 5)
  {
    vbattery_avg[vmeasurements_counter] = vvalue;
  }
  else
  {
    //accumulate the 5 values
    for(uint8_t i=0; i<VOLTAGE_AVG_SIZE; i++)
    {
      voltage_accumulator = voltage_accumulator + vbattery_avg[i];
    }
    //do the average
    float voltage_avg = voltage_accumulator / VOLTAGE_AVG_SIZE;

    for(uint8_t i=0; i<VOLTAGE_AVG_SIZE; i++)
    {
      //check if every reading is not 1 volt up or down the average
      if((vbattery_avg[i] >= (voltage_avg + 1)) || )
    }

    vmeasurements_counter = 0;
  }
  /*this function does the math calculation in order to remove trash data
  coming from both voltage sensors, it returns a variable that will be overwritten 
  until the publishing timer expires and its send to bigquery
  
  if(restart_voltage_filtering == true) //Data will be filtered and averaged again after each publication.
  {
    //restart the data accumulation and average it again
    restart_voltage_filtering = false;
  }
}
*/


// ----------------------- SENSOR DATA PUBLISHING FUNCTIONS ----------------------------

String time_calculation (void)
{
  timeClient.update();
  String currentTime = timeClient.getFormattedTime();
  return currentTime;
}

String date_calculation (void)
{
  unsigned long epochTime = timeClient.getEpochTime();
  int year_ = year(epochTime);
  int month_ = month(epochTime);
  int day_ = day(epochTime);
  current_day = day_;
  String Date_ = String(year_) + "-" + String(month_) + "-" + String(day_);
  return Date_;
}

String folio_calculation (void)
{

  //every single day the folio accumulator restarts
  if(last_day == current_day)
  {
    folio_accumulation = folio_accumulation + 1;
  }
  else
  {
    folio_accumulation = 0;
    last_day = current_day;
  }

  //transforming the counter into a 3 digits value always
  char threedigits[4];
  sprintf(threedigits, "%03d", folio_accumulation);

  String folio_ = date_calculation() + String(clave_disp) + String(threedigits);
  return folio_;
}

String usuario_cargando (void)
{
  String usuario_cargando_ = String(user_charging); 
  user_charging = false; 
                        /* restart this variable so that POLL SUBMITS MONITORING AND CHARGE CONTROL 
                              writes it again in true if somebody connected his phone before next data publish*/
  return usuario_cargando_;
}
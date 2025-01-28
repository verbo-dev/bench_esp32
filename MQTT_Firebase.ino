/*MACROS for debugging:
define DEBUGGING_OFF if you want to test the whole program
define "FEATURE"_DEBUGGING in order to just run a feature section in the program with some extra print lines
you can define multiple FEATURE_DEBUGGING macros*/
#define DEBUGGING_OFF
//#define POLLnCHARGE_DEBUGGING //Poll submits monitoring and charge control
//#define VOLTAGE_DEBUGGING //voltage and sensor processing
//#define BIGQ_DEBUGGING //sensor data publishing


#include <Arduino.h>
#include <WiFi.h>
//firebase libraries
#include <FirebaseESP32.h>
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
#define API_KEY "VtEm9kyvlhFnYa5spqfCnIKHopUKIBMvVraGvuCH"
// Insert RTDB URLefine the RTDB URL */
#define DATABASE_URL "https://verbo-app-1e648-default-rtdb.firebaseio.com/"
//Define Firebase Data object
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
void firebase_config(void);


// ---- Big Query (MQTT connection) -----------
//mqtt server connection
PubSubClient mqttClient(esp32Client);
#define server "34.66.172.243" //remote google computer server
#define port 1883
const char * la_caldera_logger_t = "la_caldera/Bench1";
void mqtt_connect(void);
void callback(char*, byte*, unsigned int);

//for callback function
int var = 0;
String resultS = "";

// ----- poll submits monitoring and charge control variables -----
bool signupOK = false;
bool get_cargador_status(void); //read from firebase 
bool CargadorStatus = false;
bool monitor_connection(void);
void end_sessionFb(void);
bool user_charging = false;
//current sensor object
Adafruit_INA219 ina219;
void loadConfigFromFirebase(void);
String currentSessionId = "";
void ping_Fb(void);

// ----- general timers -----------
#define NO_WIFI_MODE_ACTIVATION_t 83 // NO_WIFI_MODE_ACTIVATION = NO_WIFI_MODE_ACTIVATION_t * internal function delay
#define BIGQUERY_PUBLISHING_t 720 //BIGQUERY_PUBLISHING = BIGQUERY_PUBLISHING_t (in seconds) / global void loop delay (in seconds)
uint8_t USER_NOT_CHARGING_TIMEOUT_t = 0; //USER_NOT_CHARGING_TIMEOUT = USER_NOT_CHARGING_TIMEOUT_t * global void loop delay
#define GLOBAL_VOID_LOOP_DELAY 500 //ms 


// ------ voltage and sensor processing variables -------------
float voltage_panel = 0.0;
float voltage_battery = 0.0;
float voltage_battery_filtered = 0.0;
float voltage_panel_filtered = 0.0;
float filtering(float);
#define RELAY_PIN 5
//sensor voltaje:::::::::::::
#define ANALOG_PIN_PANEL 36
#define ANALOG_PIN_BATTERY 34
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
  Serial.begin(115200);
  //:::::::::::::::::::::::::::

  // -------------------------------- CONNECTION TO WIFI ---------------------------------------------

  NO_WIFI_MODE = wifi_begin();

  // ------------------------------ CONNECTION TO FIREBASE ----------------------------------------

  #if defined(POLLnCHARGE_DEBUGGING) || defined(DEBUGGING_OFF)
    if(NO_WIFI_MODE == false)
    {
      firebase_config();
    }
  #endif
  
  // ------------------------------CONNECTION TO BIG QUERY (MQTT)------------------------------------
  #if defined(BIGQ_DEBUGGING) || defined(DEBUGGING_OFF)
    if(NO_WIFI_MODE == false)
    {
      mqttClient.setServer(server, port);
      mqttClient.setKeepAlive((BIGQUERY_PUBLISHING_t / 2)+10); // +10 just to have a gap for the keepalive
      mqttClient.setCallback(callback);
      #ifdef BIGQ_DEBUGGING
        Serial.println("DEBUG:MQTT connection parameters ");
        Serial.println(server);
        Serial.println((BIGQUERY_PUBLISHING_t / 2)+10);
      #endif

      mqtt_connect();
    }
  #endif
  // initialize digital pin GPIO18 as an output.
  pinMode(RELAY_PIN, OUTPUT);
  
  //for sensor data publishing to calculate date and time
  timeClient.begin();
  timeClient.setTimeOffset(-21600); //para llegar a una zona horaria GMT-6

  //I2C communication with current sensor begins
  ina219.begin(); //pins 21 and 22 of ESP32 as default

}


// the loop function runs over and over again forever
void loop() {


  // ------ POLL SUBMITS MONITORING AND CHARGE CONTROL ------------
  #if defined(POLLnCHARGE_DEBUGGING) || defined(DEBUGGING_OFF)
    if ((Firebase.ready() && signupOK) && (NO_WIFI_MODE == false)) //if everything is ok in firebase
    {
      if(CargadorStatus == true)
      {
        CargadorStatus = monitor_connection(); //check if the user is connected to the chargers using current sensor
        if(CargadorStatus == false)
        {
          #ifdef POLLnCHARGE_DEBUGGING
            Serial.println("DEBUG:writting a false to firebase and turning off chargers");
          #endif
          end_sessionFb(); 
          digitalWrite(RELAY_PIN, LOW); //turn off the chargers
        }
        
      }
      else
      {
        CargadorStatus = get_cargador_status(); //poll if the status changed from false
        #ifdef POLLnCHARGE_DEBUGGING
          Serial.println("DEBUG:checking if somebody wants to connect");
        #endif
        if(CargadorStatus == true)
        {
          #ifdef POLLnCHARGE_DEBUGGING
            Serial.println("DEBUG:somebody just connected his phone");
          #endif
          user_charging = true; //for sensor data publishing to know that in this BIGQUERY_PUBLISHING time lapse somebody was connected
          digitalWrite(RELAY_PIN, HIGH); //turn on the chargers
        }
      }   
    }
    else //chargers will be always working if there is no internet, no firebase connection or firebase says that we are in developer mode
    {
      digitalWrite(RELAY_PIN, HIGH); 
    }
  #else
    digitalWrite(RELAY_PIN, HIGH);
  #endif

  // --------------- VOLTAGE AND SENSOR PROCESSING ----------------------------
  #if defined(VOLTAGE_DEBUGGING) || defined(DEBUGGING_OFF)
    voltage_battery = voltage_measuring(ANALOG_PIN_BATTERY);
    //voltage_battery_filtered = filtering(voltage_battery);


    voltage_panel = voltage_measuring(ANALOG_PIN_PANEL);
    //voltage_panel_filtered = filtering(voltage_panel);

    #ifdef VOLTAGE_DEBUGGING
      Serial.print("DEBUG:final panel voltage value ");
      Serial.println(voltage_panel);
      Serial.print("DEBUG:final battery voltage value ");
      Serial.println(voltage_battery);
      Serial.print("DEBUG:current value ");
      Serial.println(current_calculation());
    #endif
  #endif

    // --------------- WIFI TROUBLESHOOTING --------------------------------------
  if(WiFi.status() != WL_CONNECTED) //check if the connection to internet is still stable 
  {
    NO_WIFI_MODE = wifi_begin();
    if(NO_WIFI_MODE == false) //reconnect to everything if there is wifi connection again
    {
      #if defined(POLLnCHARGE_DEBUGGING) || defined(DEBUGGING_OFF)
        firebase_config();
      #endif
      #if defined(BIGQ_DEBUGGING) || defined(DEBUGGING_OFF)
      if (!mqttClient.connected())
        mqtt_connect(); //this function will BLOCK the program if connection with google remote computer is not succesfull
      #endif
    }
  }

  // --------------- SENSOR DATA PUBLISHING --------------------------------------
  #if defined(BIGQ_DEBUGGING) || defined(DEBUGGING_OFF)
    bigquery_publishing_timeout = bigquery_publishing_timeout - 1;

    #ifdef BIGQ_DEBUGGING
      Serial.print("DEBUG:bigquery_publishing_timeout ");
      Serial.println(bigquery_publishing_timeout);
    #endif

    //data will be published only if timer expired or somebody just answered the poll
    if(((bigquery_publishing_timeout == 0) && (NO_WIFI_MODE == false)) || (user_charging == true))
    {
      bigquery_publishing_timeout = BIGQUERY_PUBLISHING_t;
        //prepare variables to be published on mqtt
      String voltage_panel_filteredS = String(voltage_panel);
      String voltage_battery_filteredS = String(voltage_battery);

      String DatatoPublish = voltage_panel_filteredS + "|" + voltage_battery_filteredS + "|" + clave_disp \
                            + "|" + date_calculation() + "|" + time_calculation() + "|" + folio_calculation() \
                            + "|" + usuario_cargando() + "|" + get_sID(); //added the last string for sesion id

      //publish in mqtt 
      mqttClient.publish(la_caldera_logger_t,DatatoPublish.c_str(),false); //retain set to false
      Serial.println("publishing this data:" + DatatoPublish + "into topic:");
      Serial.print(la_caldera_logger_t);
    }
    else if (NO_WIFI_MODE == true)
    {
      //logic to save the sensor data in a separate variable 
    }
  #endif
  #if defined(BIGQ_DEBUGGING) || defined(DEBUGGING_OFF)
    //maintain alive the connection and poll for more incoming messages
    mqttClient.loop();
  #endif
  #if defined(POLLnCHARGE_DEBUGGING) || defined(DEBUGGING_OFF)
    ping_Fb();
  #endif
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
  config.signer.tokens.legacy_token = API_KEY;

  /* Assign the RTDB URL (required) */
  config.host = DATABASE_URL;

  /* Sign up */
  Firebase.begin(&config, &auth);
  signupOK = true;
  fbdo.setBSSLBufferSize(2048, 512);
  fbdo.setResponseSize(2048);
  loadConfigFromFirebase();
}

void mqtt_connect() 
{
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

  /*resultS = "";
  
  for (int i=0;i<length;i++) {
    resultS= resultS + (char)payload[i];
  }
  Serial.println(resultS);
  */
}

// ------------------------- SENSOR PROCESSING AND POLL SUBMITS MONITORING FUNCTIONS ----------------------

// read the USER_NOT_CHARGING_TIMEOUT written in firebase
void loadConfigFromFirebase() {
  String path = "/benches/" + String(clave_disp) + "/config";
  if (Firebase.getJSON(fbdo, path)) {
    FirebaseJson json = fbdo.jsonObject();
    FirebaseJsonData fbdoJSON;
    int sensorTimeout;
    if (json.get(fbdoJSON, "sensorTimeoutDuration") && fbdoJSON.type == "int") {
      //data from firebase is sent in seconds
      USER_NOT_CHARGING_TIMEOUT_t = (fbdoJSON.intValue * 1000) / GLOBAL_VOID_LOOP_DELAY; 
    }
    Serial.println("Configurable timers loaded");
  } else {
    Serial.println("Firebase configuration error, setting default timer");
    USER_NOT_CHARGING_TIMEOUT_t = 60;
    Serial.println(fbdo.errorReason());
  }
}


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
    #ifdef VOLTAGE_DEBUGGING
      Serial.print("DEBUG:counts recieved for battery ");
      Serial.println(adc_value);
    #endif

    return voltage;
  }

  #ifdef VOLTAGE_DEBUGGING
    Serial.print("DEBUG:counts recieved for panel ");
    Serial.println(adc_value);
  #endif
  //Voltage measuring from panel has an extra voltage divider
  return (voltage * (R1_PCB + R2_PCB)) / R2_PCB;
}

//read if /activeSession topic in firebase has some string data
bool get_cargador_status(void)
{
  bool CargadorStatus = false;
  if (Firebase.getString(fbdo, "/benches/" + String(clave_disp) + "/activeSession"))
  {
    currentSessionId = fbdo.stringData();
    #ifdef POLLnCHARGE_DEBUGGING
      Serial.println("DEBUG:active session recieved");
    #endif
    return true;
  } 
  return false;
}

//check if there are cellphones connected 
bool monitor_connection(void)
{
  float current_value = current_calculation();

  #ifdef POLLnCHARGE_DEBUGGING
    Serial.print("DEBUG:current value ");
    Serial.println(current_value);
    Serial.print("DEBUG:NO_USERS_CHARGING_THRESHOLDVAL ");
    Serial.println(user_not_charging_timeout);
  #endif

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

//let know firebase that there is no one charging and register the time when that user stopped charging
void end_sessionFb(void)
{
  String endTimePath = "/sessions/" + currentSessionId + "/endTime";
  String activeSessionPath = "/benches/" + String(clave_disp) + "/activeSession";

  Firebase.setString(fbdo, endTimePath, time_calculation());
  Firebase.setString(fbdo, activeSessionPath, "null");

  currentSessionId = "";
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

String get_sID(void)
{
  return currentSessionId;
  //develop the code that reads from firebase the session id and return it in string "session123", not all the path
  //fbdo is the object name in this code that has the reading from firebase methods "fbdo.boolData()" for example, it replaced firebaseData of your other code 
}

//keep alive the connection with firebase 
void ping_Fb(void)
{
  uint8_t feedback = 0;
  String path = "/benches/" + String(clave_disp) + "/lastConnection";
  String currentTime = time_calculation();
  feedback = Firebase.setString(fbdo, path, currentTime);

  #ifdef POLLnCHARGE_DEBUGGING
    if (feedback) {
      Serial.println("DEBUG:Ping correctly written");
    } else {
      Serial.println("DEBUG:Error while doing a ping to firebase:");
      Serial.println(firebaseData.errorReason());
    }
  #endif
}
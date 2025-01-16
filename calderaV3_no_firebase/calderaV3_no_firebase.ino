#include <WiFi.h>
#include <PubSubClient.h>





//RELAY
#define PIN_RELAY_1  27 // The ESP32 pin GPIO27 connected to the IN1 pin of relay module
#define PIN_RELAY_2  26 // The ESP32 pin GPIO26 connected to the IN2 pin of relay module
//:::::::::::::::

//WiFi connection
WiFiClient esp32Client;
#define ssid "caldera staff"
#define password "LCStaff20+&"

//sensor voltaje:::::::::::::
#define V_IN_1  36  //  PIN GPIO36  SENSOR 
#define V_IN_2  37  //  PIN GPIO037 
#define REF_VOLTAGE    3.3
#define ADC_RESOLUTION 4096.0
#define R1             29800.0 // resistor values in voltage sensor (in ohms)
#define R2               7470.0  // resistor values in voltage sensor (in ohms)

//hora fecha :::::::::::::::::::::::::
#include <NTPClient.h>
#include <TimeLib.h>
//server works with UDP protocol
WiFiUDP ntpUDP;
//page of the server 
NTPClient timeClient(ntpUDP, "pool.ntp.org");

//date related variables
unsigned long epochTime;
String currentTime ="";
int currentYear;  
int currentMonth;
int currentDay;
String currentDate;

//:::::::::::::::

//mqtt server connection
PubSubClient mqttClient(esp32Client);
#define server "34.66.172.243"
#define port 1883
const char* topic1 = "la_caldera/Bench1";
const char* topic2 = "la_caldera/Bench2";
const char* topic_disp = "dispositivos";
const char* topic_clientes = "clientes";

//mqtt callback function
String resultS = "";
int var = 0;

//functions declaration
void wifiInit();
void callback(char*,byte*,unsigned int);
void reconnect();

// TBL_LOGGER
float voltaje_panel = 12.59;
float voltaje_bateria = 13.05; // Corrected typo from "FOAT" to "FLOAT"
int clave_disp = 1;
String date = ""; // Using String to represent date
String time_ = ""; // Using String to represent time
int folio = clave_disp + 1;
bool usuario_cargando = false; // BOOLEAN type in Arduino

String voltaje_panelS;
String voltaje_bateriaS;
String clave_dispS;
String folioS;
String usuario_cargandoS;

String concatenated;
void setup() {
  Serial.begin(115200);  // Initialize serial communication at 115200 baud
  //wifi connection
  delay(10);
  wifiInit();
  //mqtt server connection
  mqttClient.setServer(server, port);
  mqttClient.setKeepAlive(360);
  mqttClient.setCallback(callback);

  //SENSOR VOLTAJE::: 
   // set the ADC attenuation to 11 dB (up to ~3.3V input)
  analogSetAttenuation(ADC_11db);
  //:::::::::::::::::::::::::::

  //obtener hora::::::::::::::::::
    timeClient.begin();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  timeClient.begin();
  timeClient.setTimeOffset(-21600); //para llegar a una zona horaria GMT-6

  //RELAY
  pinMode(PIN_RELAY_1, OUTPUT);
  pinMode(PIN_RELAY_2, OUTPUT);
  //::::::::::::

  
}

void loop() {
  //hora::::::::::::::::::::::
  timeClient.update();
  //time
  currentTime = timeClient.getFormattedTime();
  Serial.println(currentTime);
  //date
  epochTime = timeClient.getEpochTime();

  currentYear = year(epochTime); 
  currentMonth = month(epochTime);
  currentDay = day(epochTime);

  currentDate = String(currentYear) + "-" + String(currentMonth) + "-" + String(currentDay);

  Serial.println(currentDate);
  String date = currentDate; // Using String to represent date
  String time_ = currentTime;
  //FIN HORA::::::::::::::::::::
  

  
  // :::::::::::SENSOR VOLTAJE
  
  //read the analog input 1
  int adc_value = analogRead(V_IN_1);

  // determine voltage at adc input
  float voltage_adc = ((float)adc_value * REF_VOLTAGE) / ADC_RESOLUTION;

  // calculate voltage at the sensor input
  float voltage_in1 = voltage_adc * (R1 + R2) / R2;

  //read the analog input 1
  adc_value = analogRead(V_IN_2);

  // determine voltage at adc input
  voltage_adc = ((float)adc_value * REF_VOLTAGE) / ADC_RESOLUTION;

  // calculate voltage at the sensor input
  float voltage_in2 = voltage_adc * (R1 + R2) / R2;

  //fin voltaje






    delay(10000);

    if (!mqttClient.connected()) {
      reconnect();
    }
    //maintain alive the connection and poll for more incoming messages
    mqttClient.loop();

    /*
    //it publishes to the dispositivos table just one to register
    mqttClient.publish(topic_disp,"0|4321|2024-08-10|0001|62.5|84.1",false);

    //it publishes to clientes one time just to register
    delay(10000);
    mqttClient.publish(topic_clientes,"0002|la_caldera|2024-08-10",false);
    */


  //set initial mqtt connection
  if (!mqttClient.connected()) {
    reconnect();
  }
  //maintain alive the connection and poll for more incoming messages
  mqttClient.loop();
  
  voltaje_panelS = String(voltage_in1);
  voltaje_bateriaS = String(voltage_in2);
  clave_dispS = String(clave_disp);
  folioS = String(folio);
  usuario_cargandoS = String(usuario_cargando);

  concatenated = voltaje_panelS + "|" + voltaje_bateriaS + "|" + clave_dispS + "|" + date + "|" + time_ + "|" + folioS + "|" + usuario_cargandoS;
  
  //retain set to TRUE
  mqttClient.publish(topic1,concatenated.c_str(),false);
  Serial.println("publishing in topic1: " + concatenated);

  folio = folio + 1;

  digitalWrite(PIN_RELAY_1, HIGH);
  digitalWrite(PIN_RELAY_2, HIGH);
  delay(5000);

  Serial.println("Turn off both relays");
  digitalWrite(PIN_RELAY_1, LOW);
  digitalWrite(PIN_RELAY_2, LOW);

  delay(10000);
  //delay(10000);

}

void wifiInit() {
  Serial.println("Entrando a función WifiInit");
  Serial.print("Conectándose a ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
      delay(500);  
  }
  Serial.println("");
  Serial.println("Conectado a WiFi");
  Serial.println("Dirección IP: ");
  Serial.println(WiFi.localIP());
  Serial.println("Turn on both relays");


  } 

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.println("entrando en función callbak");
  Serial.print("Mensaje recibido [");
  Serial.print(topic);
  Serial.print("] ");

  char payload_string[length + 1];
  
  int resultI;

  memcpy(payload_string, payload, length);
  payload_string[length] = '\0';
  resultI = atoi(payload_string);
  
  var = resultI;

  resultS = "";
  
  for (int i=0;i<length;i++) {
    resultS= resultS + (char)payload[i];
  }
  Serial.println(resultS);
}

void reconnect() {
  while (!mqttClient.connected()) {
    Serial.print("Intentando conectarse MQTT...");

    if (mqttClient.connect("arduinoClient")) {
      Serial.println("Conectado");

      mqttClient.subscribe(topic1);
      //mqttClient.subscribe(topic2);
      
    } else {
      Serial.print("Fallo, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" intentar de nuevo en 5 segundos");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}



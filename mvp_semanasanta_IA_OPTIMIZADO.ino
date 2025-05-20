#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_INA219.h>

// ----- Instancias -----
Adafruit_INA219 sensorUSB(0x44);
Adafruit_INA219 sensorInalambrico(0x40);
Preferences nvmlib;

// ----- Calibraciones -----
float idleCurrent1 = 0, oneDeviceCurrent1 = 0, twoDeviceCurrent1 = 0;
float idleCurrent2 = 0, oneDeviceCurrent2 = 0, twoDeviceCurrent2 = 0;

float threshold1_1 = 500, threshold1_2 = 1000;
float threshold2_1 = 500, threshold2_2 = 1000;

bool calibrated = false;

// ----- Estado actual -----
int state1 = 0; // 0, 1 o 2 dispositivos
int state2 = 0;

int usersCount1 = 0;
int usersCount2 = 0;
int noiseCounter1 = 0;
int noiseCounter2 = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!sensorUSB.begin()) {
    Serial.println("Sensor cargadores USB (0x44) no detectado");
    while (true);
  }

  if (!sensorInalambrico.begin()) {
    Serial.println("Sensor cargadores inalámbricos (0x40) no detectado");
    while (true);
  }

  Serial.println("Sensores inicializados.");

  nvmlib.begin("storage", false);

  // Restaurar umbrales si existen
  threshold1_1 = nvmlib.getFloat("t1_1", 500);
  threshold1_2 = nvmlib.getFloat("t1_2", 1000);
  threshold2_1 = nvmlib.getFloat("t2_1", 500);
  threshold2_2 = nvmlib.getFloat("t2_2", 1000);

  usersCount1 = nvmlib.getInt("usersCount1", 0);
  usersCount2 = nvmlib.getInt("usersCount2", 0);

  calibrated = true;

  Serial.println("Umbrales restaurados:");
  Serial.println("USB:   0→1: " + String(threshold1_1) + " | 1→2: " + String(threshold1_2));
  Serial.println("INAL: 0→1: " + String(threshold2_1) + " | 1→2: " + String(threshold2_2));
  Serial.println("Conteos acumulados restaurados:");
  Serial.println("[USB] " + String(usersCount1));
  Serial.println("[INALÁMBRICO] " + String(usersCount2));
}

void loop() 
{
  float current1 = -sensorUSB.getCurrent_mA();
  float current2 = -sensorInalambrico.getCurrent_mA();

  // Estado sensor USB
  if (current1 < threshold1_1) 
  {
      if (state1 != 0) Serial.println("[USB] Sin dispositivos conectados.");
      state1 = 0;
  } 
  
  else if (current1 < threshold1_2) 
  {
    noiseCounter1++;
    if(state1 == 2) //if current decremented just once
    {
      state1 = 1; //we transition from 2 phones charging to just 1
      Serial.println("[USB] 1 dispositivo conectado.");
    }
    else if (state1 == 0) 
    { //if current incremented 3 times
      if(noiseCounter1 == 5)
      {
        state1 = 1;
        usersCount1++;
        nvmlib.putInt("usersCount1", usersCount1);
        Serial.println("[USB] 1 dispositivo conectado.");
      }
    }
    else 
    {
      noiseCounter1 = 0; //we just confirm that it is in state 1 
    }
  }

 else //above threshold 2 (2 phones) 
 {
    noiseCounter1++;
    Serial.println("[USB] 2 dispositivos conectados.");
    if (state1 == 1) 
    {
      if(noiseCounter1 == 5)
      {
        state1 = 2;
        usersCount1++;
        nvmlib.putInt("usersCount1", usersCount1);
      }
    }
    else if(state1 == 2)
    {
      noiseCounter2 == 0;
    }
  }
    
  
  

  // Estado sensor Inalámbrico
  if (current2 < threshold2_1) 
  {
      if (state2 != 0) Serial.println("[INAL] Sin dispositivos conectados.");
      state2 = 0;
  } 
  
  else if (current2 < threshold2_2) 
  {
    noiseCounter2++;
    if(state2 == 2) //if current decremented just once
    {
        state2 = 1; //we transition from 2 phones charging to just 1
        Serial.println("[INAL] 1 dispositivo conectado.");
    }
    else if (state2 == 0) 
    { //if current incremented 3 times
      if(noiseCounter2 == 5)
      {
        state2 = 1;
        usersCount2++;
        nvmlib.putInt("usersCount2", usersCount2);
        Serial.println("[INAL] 1 dispositivo conectado.");
      }
    }
    else 
    {
      noiseCounter2 = 0; //we just confirm that it is in state 1 
    }
  }

 else //above threshold 2 (2 phones) 
 {
    noiseCounter2++;
    Serial.println("[INAL] 2 dispositivos conectados.");
    if (state2 == 1) 
    {
      if(noiseCounter2 == 5)
      {
        state2 = 2;
        usersCount2++;
        nvmlib.putInt("usersCount2", usersCount2);
      }
    }
    else if(state2 == 2)
    {
      noiseCounter2 == 0;
    }
  }

  // Comandos por Serial
  if (Serial.available()) {
    String comando = Serial.readStringUntil('\n');
    comando.trim();

    if (comando.startsWith("u,")) {
      int firstComma = comando.indexOf(',');
      int secondComma = comando.indexOf(',', firstComma + 1);

      if (firstComma > 0 && secondComma > firstComma) {
        int tipo = comando.substring(firstComma + 1, secondComma).toInt();
        float valor = comando.substring(secondComma + 1).toFloat();

        switch (tipo) {
          case 11:
            threshold1_1 = valor;
            nvmlib.putFloat("t1_1", threshold1_1);
            Serial.println("Umbral USB 0→1 actualizado a " + String(valor) + " mA");
            break;
          case 12:
            threshold1_2 = valor;
            nvmlib.putFloat("t1_2", threshold1_2);
            Serial.println("Umbral USB 1→2 actualizado a " + String(valor) + " mA");
            break;
          case 21:
            threshold2_1 = valor;
            nvmlib.putFloat("t2_1", threshold2_1);
            Serial.println("Umbral INAL 0→1 actualizado a " + String(valor) + " mA");
            break;
          case 22:
            threshold2_2 = valor;
            nvmlib.putFloat("t2_2", threshold2_2);
            Serial.println("Umbral INAL 1→2 actualizado a " + String(valor) + " mA");
            break;
          default:
            Serial.println("Tipo inválido. Usa 11, 12, 21 o 22.");
        }
      } else {
        Serial.println("Formato inválido. Usa: u,11,valor");
      }
    } else if (comando == "a") {
      idleCurrent1 = -sensorUSB.getCurrent_mA();
      Serial.println("[USB] Corriente en reposo: " + String(idleCurrent1) + " mA");
    } else if (comando == "b") {
      oneDeviceCurrent1 = -sensorUSB.getCurrent_mA();
      Serial.println("[USB] Corriente con 1 dispositivo: " + String(oneDeviceCurrent1) + " mA");
    } else if (comando == "c") {
      twoDeviceCurrent1 = -sensorUSB.getCurrent_mA();
      Serial.println("[USB] Corriente con 2 dispositivos: " + String(twoDeviceCurrent1) + " mA");
    } else if (comando == "d") {
      idleCurrent2 = -sensorInalambrico.getCurrent_mA();
      Serial.println("[INALÁMBRICO] Corriente en reposo: " + String(idleCurrent2) + " mA");
    } else if (comando == "e") {
      oneDeviceCurrent2 = -sensorInalambrico.getCurrent_mA();
      Serial.println("[INALÁMBRICO] Corriente con 1 dispositivo: " + String(oneDeviceCurrent2) + " mA");
    } else if (comando == "f") {
      twoDeviceCurrent2 = -sensorInalambrico.getCurrent_mA();
      Serial.println("[INALÁMBRICO] Corriente con 2 dispositivos: " + String(twoDeviceCurrent2) + " mA");
    } else if (comando == "x") {
      threshold1_1 = (idleCurrent1 + oneDeviceCurrent1) / 2.0;
      threshold1_2 = (oneDeviceCurrent1 + twoDeviceCurrent1) / 2.0;
      threshold2_1 = (idleCurrent2 + oneDeviceCurrent2) / 2.0;
      threshold2_2 = (oneDeviceCurrent2 + twoDeviceCurrent2) / 2.0;

      nvmlib.putFloat("t1_1", threshold1_1);
      nvmlib.putFloat("t1_2", threshold1_2);
      nvmlib.putFloat("t2_1", threshold2_1);
      nvmlib.putFloat("t2_2", threshold2_2);

      Serial.println("Umbrales recalculados y guardados:");
      Serial.println("USB:   0→1: " + String(threshold1_1) + " | 1→2: " + String(threshold1_2));
      Serial.println("INAL: 0→1: " + String(threshold2_1) + " | 1→2: " + String(threshold2_2));

      calibrated = true;
    } else if (comando == "r") {
      Serial.println("Conteo acumulado:");
      Serial.println("[USB] Dispositivos registrados: " + String(usersCount1));
      Serial.println("[INALÁMBRICO] Dispositivos registrados: " + String(usersCount2));
    } else if (comando == "z") {
      static int confirm = 0;
      confirm++;
      Serial.println("¿Deseas borrar los conteos? Presiona 'z' nuevamente para confirmar o 'r' para cancelar.");
      if (confirm == 2) {
        usersCount1 = 0;
        usersCount2 = 0;
        nvmlib.putInt("usersCount1", usersCount1);
        nvmlib.putInt("usersCount2", usersCount2);
        Serial.println("Conteo acumulado reiniciado.");
        confirm = 0;
      }
    } else if (comando == "s") {
      Serial.println("--- Estado de sensores y umbrales ---");
      Serial.println("[USB] Corriente actual: " + String(current1) + " mA");
      Serial.println("Umbral 0→1: " + String(threshold1_1) + " | 1→2: " + String(threshold1_2));
      Serial.println("[INALÁMBRICO] Corriente actual: " + String(current2) + " mA");
      Serial.println("Umbral 0→1: " + String(threshold2_1) + " | 1→2: " + String(threshold2_2));
    }
    
  }
  delay(500);
}
#include <DHT.h>
#include <DHT_U.h>
#include <Wire.h>

#include <ArduinoJson.h>
#include <ArduinoJson.hpp>

#include <math.h>

#define DHTTYPE DHT22

#define FOGTEMPHUM_PIN 4
#define PLANTTEMPHUM_PIN 5
#define FOGTDS_PIN A1
#define TANKTDS_PIN A2

#define PUMP_PIN 6
#define FOGGER1_PIN 7
#define FOGGER2_PIN 8
#define LIGHT_PIN 9


#define VREF 5.0 // analog reference voltage(Volt) of the ADC
#define SCOUNT  20 // sum of sample point

#define NO_TOUCH       0xFE

#define THRESHOLD      250

#define ATTINY1_HIGH_ADDR   0x78
#define ATTINY2_LOW_ADDR   0x77

int fogtds_analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int tanktds_analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int fogtds_analogBufferTemp[SCOUNT];
int tanktds_analogBufferTemp[SCOUNT];
int copyIndex = 0;
int fogtds_analogBufferIndex = 0;
int tanktds_analogBufferIndex = 0;
float fogtds_averageVoltage = 0,fogtds_value = 0;
float tanktds_averageVoltage = 0,tanktds_value = 0;
unsigned char low_data[8] = {0};
unsigned char high_data[12] = {0};

float fogtemp;
float foghum;
float planttemp;
float planthum;
int fogwl = 0;

bool pump_state = false;
bool fogger1_state = false;
bool fogger2_state = false;
bool light_state = true;

float log_2 = log(2);

DHT fogtemphum_sensor(FOGTEMPHUM_PIN, DHTTYPE);
DHT planttemphum_sensor = DHT(PLANTTEMPHUM_PIN, DHTTYPE);



void setup() {
  Serial.begin(115200);
  Wire.begin();
  fogtemphum_sensor.begin();
  planttemphum_sensor.begin();
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(FOGGER1_PIN, OUTPUT);
  pinMode(FOGGER2_PIN, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);
  delay(2000);
}

void trigger_pump (bool desired_state) {
  if (desired_state)
  {
    digitalWrite(PUMP_PIN, HIGH);
  }
  else
  {
    digitalWrite(PUMP_PIN, LOW);
  }
  pump_state = desired_state;
}

void trigger_fogger1(bool desired_state) {
  if (desired_state)
  {
    digitalWrite(FOGGER1_PIN, HIGH);
  }
  else
  {
    digitalWrite(FOGGER1_PIN, LOW);
  }
  fogger1_state = desired_state;
}

void trigger_fogger2(bool desired_state) {
  if (desired_state)
  {
    digitalWrite(FOGGER2_PIN, HIGH);
  }
  else
  {
    digitalWrite(FOGGER2_PIN, LOW);
  }
  fogger2_state = desired_state;
}

void trigger_light(bool desired_state)
{
  if (desired_state)
  {
    digitalWrite(LIGHT_PIN, LOW);
  }
  else
  {
    digitalWrite(LIGHT_PIN, HIGH);
  }
  light_state = desired_state;
}

void read_actuators() {
  //Pump
  StaticJsonDocument<16> pumpstate;
  pumpstate["actuator"] = "Pump";
  pumpstate["value"] = pump_state;
  serializeJson(pumpstate, Serial);
  Serial.println();
  //Fogger1
  StaticJsonDocument<16> fogger1state;
  fogger1state["actuator"] = "Fogger1";
  fogger1state["value"] = fogger1_state;
  serializeJson(fogger1state, Serial);
  Serial.println();
  //Fogger2
  StaticJsonDocument<16> fogger2state;
  fogger2state["actuator"] = "Fogger2";
  fogger2state["value"] = fogger2_state;
  serializeJson(fogger2state, Serial);
  Serial.println();
  //Light
  StaticJsonDocument<16> lightstate;
  lightstate["actuator"] = "Light";
  lightstate["value"] = light_state;
  serializeJson(lightstate, Serial);
  Serial.println();
}

void sample_fogtds() {
  fogtds_analogBuffer[fogtds_analogBufferIndex] = analogRead(FOGTDS_PIN);    //read the analog value and store into the buffer
  fogtds_analogBufferIndex++;
  if(fogtds_analogBufferIndex == SCOUNT)
  {
    fogtds_analogBufferIndex = 0;
  }
}

float read_fogtds() {
  for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
  {
    fogtds_analogBufferTemp[copyIndex]= fogtds_analogBuffer[copyIndex];
  }
  fogtds_averageVoltage = getMedianNum(fogtds_analogBufferTemp,SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
  if (isnan(fogtemp))
  {
    fogtemp = 25;
  }
  float compensationCoefficient=1.0+0.02*(fogtemp-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
  float fogtds_compensationVoltage=fogtds_averageVoltage/compensationCoefficient;  //temperature compensation
  fogtds_value=(133.42*fogtds_compensationVoltage*fogtds_compensationVoltage*fogtds_compensationVoltage - 255.86*fogtds_compensationVoltage*fogtds_compensationVoltage + 857.39*fogtds_compensationVoltage)*0.5; //convert voltage value to tds value
  return fogtds_value;
}

void sample_tanktds() {
  tanktds_analogBuffer[tanktds_analogBufferIndex] = analogRead(TANKTDS_PIN);    //read the analog value and store into the buffer
  tanktds_analogBufferIndex++;
  if(tanktds_analogBufferIndex == SCOUNT)
  {
    tanktds_analogBufferIndex = 0;
  }
}

float read_tanktds() {
  for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
  {
    tanktds_analogBufferTemp[copyIndex]= tanktds_analogBuffer[copyIndex];
  }
  tanktds_averageVoltage = getMedianNum(tanktds_analogBufferTemp,SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
  if (isnan(planttemp))
  {
    planttemp = 23;
  }
  float compensationCoefficient=1.0+0.02*(planttemp-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
  float tanktds_compensationVoltage=tanktds_averageVoltage/compensationCoefficient;  //temperature compensation
  tanktds_value=(133.42*tanktds_compensationVoltage*tanktds_compensationVoltage*tanktds_compensationVoltage - 255.86*tanktds_compensationVoltage*tanktds_compensationVoltage + 857.39*tanktds_compensationVoltage)*0.5; //convert voltage value to tds value
  return tanktds_value;
}

int read_fog_waterlevel() {
  // int sensorvalue_min = 250;
  // int sensorvalue_max = 255;
  // int low_count = 0;
  // int high_count = 0;
  uint32_t touch_val = 0;
  uint8_t trig_section = 0;
  // low_count = 0;
  // high_count = 0;
  getLow8SectionValue();
  getHigh12SectionValue();

  // for (int i = 0; i < 8; i++)
  // {
  //   if (low_data[i] >= sensorvalue_min && low_data[i] <= sensorvalue_max)
  //   {
  //     low_count++;
  //   }
  //   // if (low_count == 8)
  //   // {
  //   //   Serial.print("      ");
  //   //   Serial.print("PASS");
  //   // }
  // }
  // for (int i = 0; i < 12; i++)
  // {
  //   if (high_data[i] >= sensorvalue_min && high_data[i] <= sensorvalue_max)
  //   {
  //     high_count++;
  //   }
  //   // if (high_count == 12)
  //   // {
  //   //   Serial.print("      ");
  //   //   Serial.print("PASS");
  //   // }
  // }

  for (int i = 0 ; i < 8; i++) {
    if (low_data[i] > THRESHOLD) {
      touch_val |= 1 << i;

    }
  }
  for (int i = 0 ; i < 12; i++) {
    if (high_data[i] > THRESHOLD) {
      touch_val |= (uint32_t)1 << (8 + i);
    }
  }

  while (touch_val & 0x01)
  {
    trig_section++;
    touch_val >>= 1;
  }
  fogwl = trig_section * 5;
  //fogwl = (log(touch_val) / log_2) * 5;
  return fogwl;
}

void read_sensors() {
  // Fog Temperaturesensor and Fog Humiditysensor
  static unsigned long fogtemphum_timepoint = millis();
  if(millis()-fogtemphum_timepoint > 2000U)
  {
    fogtemphum_timepoint = millis();
    StaticJsonDocument<16> fogtemperature;
    StaticJsonDocument<16> foghumidity;

    fogtemperature["sensor"] = "FogTemperature";
    foghumidity["sensor"] = "FogHumidity";
    fogtemp = fogtemphum_sensor.readTemperature();
    foghum = fogtemphum_sensor.readHumidity();
    if (isnan(fogtemp)) {
      fogtemperature["error"] = "failed read";
    } else {
      fogtemperature["value"] = fogtemp;
    }
    if (isnan(foghum)) {
      foghumidity["error"] = "failed read";
    } else {
      foghumidity["value"] = foghum;
    }
    serializeJson(fogtemperature, Serial);
    Serial.println();
    serializeJson(foghumidity, Serial);
    Serial.println();
  }
  //Fog Temperaturesensor and Fog Humiditysensor
  static unsigned long planttemphum_timepoint = millis();
  if(millis()-planttemphum_timepoint > 2000U)
  {
    planttemphum_timepoint = millis();
    StaticJsonDocument<16> planttemperature;
    StaticJsonDocument<16> planthumidity;

    planttemperature["sensor"] = "PlantTemperature";
    planthumidity["sensor"] = "PlantHumidity";
    planttemp = planttemphum_sensor.readTemperature();
    planthum = planttemphum_sensor.readHumidity();
    if (isnan(planttemp)) {
      planttemperature["error"] = "failed read";
    } else {
      planttemperature["value"] = planttemp;
    }
    if (isnan(planthum)) {
      planthumidity["error"] = "failed read";
    } else {
      planthumidity["value"] = planthum;
    }
    serializeJson(planttemperature, Serial);
    Serial.println();
    serializeJson(planthumidity, Serial);
    Serial.println();
  }
  //FogTDSsensor
  static unsigned long fogtds_sampletimepoint = millis();
  if(millis()-fogtds_sampletimepoint > 25U)     //every 40 milliseconds,read the analog value from the ADC
  {
    fogtds_sampletimepoint = millis();
    sample_fogtds(); 
  }   
  static unsigned long fogtds_printtimeout = millis();
  if(millis()-fogtds_printtimeout > 500U)
  {
    fogtds_printtimeout = millis();
    StaticJsonDocument<16> fogtds;
    fogtds["sensor"] = "FogTDS";
    fogtds["value"] = read_fogtds();
    serializeJson(fogtds, Serial);
    Serial.println();
  }
  //TankTDSSensor
  static unsigned long tanktds_sampletimepoint = millis();
  if(millis()-tanktds_sampletimepoint > 25U)     //every 40 milliseconds,read the analog value from the ADC
  {
    tanktds_sampletimepoint = millis();
    sample_tanktds();
  }   
  static unsigned long tanktds_printtimeout = millis();
  if(millis()-tanktds_printtimeout > 500U)
  {
    tanktds_printtimeout = millis();
    StaticJsonDocument<16> tanktds;
    tanktds["sensor"] = "TankTDS";
    tanktds["value"] = read_tanktds();
    serializeJson(tanktds, Serial);
    Serial.println();
  }
  
  //FogWaterlevel
  StaticJsonDocument<16> fogwaterlevel;
  fogwaterlevel["sensor"] = "FogWaterlevel";
  Wire.beginTransmission(ATTINY1_HIGH_ADDR);
  byte fogwl_error = Wire.endTransmission();
  if (fogwl_error == 0)
  {
    fogwaterlevel["value"] = read_fog_waterlevel();
  }
  else
  {
    fogwaterlevel["error"] = "failed read";
  }
  serializeJson(fogwaterlevel, Serial);
  Serial.println();
}

void loop() {
  DynamicJsonDocument indoc(64);
  deserializeJson(indoc, Serial);
  if(!indoc.isNull())
  {
    if (indoc["actuator"] == "Pump")
    {
      trigger_pump((bool)indoc["value"]);
    }
    else if (indoc["actuator"] == "Fogger1")
    {
      trigger_fogger1((bool)indoc["value"]);
    }
    else if (indoc["actuator"] == "Fogger2")
    {
      trigger_fogger2((bool)indoc["value"]);
    }
    else if (indoc["actuator"] == "Light")
    {
      trigger_light((bool)indoc["value"]);
    }
  }

  read_sensors();
  read_actuators();
}
int getMedianNum(int bArray[], int iFilterLen) 
{
      int bTab[iFilterLen];
      for (byte i = 0; i<iFilterLen; i++)
      bTab[i] = bArray[i];
      int i, j, bTemp;
      for (j = 0; j < iFilterLen - 1; j++) 
      {
      for (i = 0; i < iFilterLen - j - 1; i++) 
          {
        if (bTab[i] > bTab[i + 1]) 
            {
        bTemp = bTab[i];
            bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
         }
      }
      }
      if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
      else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
      return bTemp;
}
void getHigh12SectionValue(void)
{
  memset(high_data, 0, sizeof(high_data));
  Wire.requestFrom(ATTINY1_HIGH_ADDR, 12);
  while (12 != Wire.available());

  for (int i = 0; i < 12; i++) {
    high_data[i] = Wire.read();
  }
  delay(10);
}


void getLow8SectionValue(void)
{
  memset(low_data, 0, sizeof(low_data));
  Wire.requestFrom(ATTINY2_LOW_ADDR, 8);
  while (8 != Wire.available());

  for (int i = 0; i < 8 ; i++) {
    low_data[i] = Wire.read(); // receive a byte as character
  }
  delay(10);
}
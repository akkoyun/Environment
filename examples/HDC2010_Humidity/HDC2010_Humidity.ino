#include "Environment.h"

// Measure Parameters
uint8_t Sensor_Read_Count = 10;
uint8_t Sensor_Average_Type = 1;

// Define Library
Environment Sensor;

void setup() {
  
  // Serial Communication Start
  Serial.begin(115200);
  
  // Start I2C
  Wire.begin();
  
  // Header
  Serial.println("     HDC2010 Humidity     ");
  Serial.println("--------------------------");

}

void loop() {
  
  // Define Sensor Variables
  float _Measurement;
  unsigned long Time;

  // Set Start Time
  Time = millis();

  // Measure
_Measurement = Sensor.HDC2010_Humidity(Sensor_Read_Count, Sensor_Average_Type);

  // Calculate Delta Time
  int DT = millis() - Time;
    
  // Serial Print Data
  Serial.print("Value         : "); Serial.print(_Measurement, 3); Serial.println(" %");
  Serial.print("Function Time : "); Serial.print(DT); Serial.println(" mS");
  Serial.println("--------------------------");
  
  // Loop Delay
  delay(1000);

}

#include "Environment.h"

void setup() {
  
  // Serial Communication Start
  Serial.begin(115200);
  
  // Start I2C
  Wire.begin();
  
  // Header
  Serial.println("     Analog Pressure      ");
  Serial.println("--------------------------");

}

void loop() {
  
  // Set Start Time
  unsigned long Time = millis();

  // Measure
  float _Measurement = Sensor.Read_Analog_Pressure(2, 10);

  // Calculate Delta Time
  long DT = millis() - Time;
    
  // Serial Print Data
  Serial.print("Pressure      : "); Serial.print(_Measurement, 3); Serial.println(" Bar");
  Serial.print("Function Time : "); Serial.print(DT); Serial.println(" mS");
  Serial.println("--------------------------");
  
  // Loop Delay
  delay(1000);

}

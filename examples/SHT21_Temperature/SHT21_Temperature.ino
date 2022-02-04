#include "Environment.h"

void setup() {
  
  // Serial Communication Start
  Serial.begin(115200);
  
  // Start I2C
  Wire.begin();
  
  // Header
  Serial.println("    SHT21 Temperature     ");
  Serial.println("--------------------------");

}

void loop() {
  
  // Set Start Time
  unsigned long Time = millis();

  // Measure
  float _Measurement = Sensor.SHT21_Temperature(5,1);

  // Calculate Delta Time
  long DT = millis() - Time;
    
  // Serial Print Data
  Serial.print("Value         : "); Serial.print(_Measurement, 3); Serial.println(" C");
  Serial.print("Function Time : "); Serial.print(DT); Serial.println(" mS");
  Serial.println("--------------------------");
  
  // Loop Delay
  delay(1000);

}

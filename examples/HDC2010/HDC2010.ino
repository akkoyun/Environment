#include "Environment.h"

// Define Sensor Object
HDC2010 _Sensor(5, true);

void setup() {

	// Serial Communication Start
	Serial.begin(115200);
  
	// Start I2C
	Wire.begin();
  
	// Header
	Serial.println("    HDC2010 T/H Sensor    ");
	Serial.println("--------------------------");

	// Set Multiplexer
	I2C.Set_Multiplexer(0x70, 3);

}

void loop() {
  
	// Set Start Time
	unsigned long Time = millis();

	// Measure
	float _Temperature = _Sensor.Temperature();
	float _Humidity = _Sensor.Humidity();

	// Calculate Delta Time
	long DT = millis() - Time;

	// Serial Print Data
	Serial.print("Temperature   : "); Serial.print(_Temperature, 3); Serial.println(" C");
	Serial.print("Humidity      : "); Serial.print(_Humidity, 3); Serial.println(" %");
	Serial.print("Function Time : "); Serial.print(DT); Serial.println(" mS");
	Serial.println("--------------------------");

	// Loop Delay
	delay(1000);

}

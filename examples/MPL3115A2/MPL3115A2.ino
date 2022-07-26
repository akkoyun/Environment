#include <Environment.h>

void setup() {

	// Serial Communication Start
	Serial.begin(115200);
  
	// Header
	Serial.println(F("    MPL3115A2 P Sensor    "));
	Serial.println(F("--------------------------"));

}

void loop() {

	// Define Sensor Object
	MPL3115A2 _Sensor(true, 3, 5, true);

	// Set Start Time
	unsigned long Time = millis();

	// Measure
	float Pressure = _Sensor.Pressure();

	// Calculate Delta Time
	long DT = millis() - Time;

	// Serial Print Data
	Serial.print(F("Pressure   : ")); Serial.print(Pressure, 4); Serial.println(F(" C"));
	Serial.print(F("Function Time : ")); Serial.print(DT); Serial.println(F(" mS"));
	Serial.println(F("--------------------------"));

	// Loop Delay
	delay(1000);

}

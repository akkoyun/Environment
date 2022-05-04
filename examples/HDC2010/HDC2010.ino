#include <Environment.h>

void setup() {

	// Serial Communication Start
	Serial.begin(115200);
  
	// Header
	Serial.println(F("    HDC2010 T/H Sensor    "));
	Serial.println(F("--------------------------"));

}

void loop() {

	// Define Sensor Object
	HDC2010 _Sensor(true, 3, 5, true);

	// Set Start Time
	unsigned long Time = millis();

	// Measure
	float Temperature = _Sensor.Temperature();
	float Humidity = _Sensor.Humidity();

	// Calculate Delta Time
	long DT = millis() - Time;

	// Serial Print Data
	Serial.print(F("Temperature   : ")); Serial.print(Temperature, 4); Serial.println(F(" C"));
	Serial.print(F("Humidity      : ")); Serial.print(Humidity, 4); Serial.println(F(" %"));
	Serial.print(F("Function Time : ")); Serial.print(DT); Serial.println(F(" mS"));
	Serial.println(F("--------------------------"));

	// Loop Delay
	delay(1000);

}

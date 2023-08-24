#include <Environment.h>

// Define Sensor Object
HDC2010 Sensor(true, __B108AA_MUX_HDC2010__);

void setup() {

	// Serial Communication Start
	Serial.begin(115200);

	// Start I2C Communication
	Sensor.Begin();

}

void loop() {

	// Set Start Time
	unsigned long Time = millis();

	// Measure
	float Temperature = Sensor.Temperature();
	float Humidity = Sensor.Humidity();

	// Calculate Delta Time
	long DT = millis() - Time;

	// Header
	Serial.println(F("    HDC2010 T/H Sensor    "));
	Serial.println(F("--------------------------"));
	Serial.print(F("Device Address : 0x")); Serial.println(Sensor.Address(), HEX);
	Serial.print(F("Mux Channel    : ")); Serial.println(Sensor.Mux_Channel());
	Serial.print(F("Device Found   : ")); Serial.println((Sensor.Detect() ? F("OK") : F("FAIL")));
	Serial.println(F("--------------------------"));

	// Serial Print Data
	Serial.print(F("Temperature    : ")); Serial.print(Temperature, 4); Serial.println(F(" C"));
	Serial.print(F("Humidity       : ")); Serial.print(Humidity, 4); Serial.println(F(" %"));
	Serial.print(F("Function Time  : ")); Serial.print(DT); Serial.println(F(" mS"));
	Serial.println(F("--------------------------"));

	// Loop Delay
	delay(1000);

}

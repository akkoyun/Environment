#include <Environment.h>

// Define Sensor Object
SDP810 Sensor(true, __B108AA_MUX_SDP810_X__);

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
	float Pressure = Sensor.Pressure(50);

	// Calculate Delta Time
	long DT = millis() - Time;

	// Header
	Serial.println(F("     SPD810 DP Sensor    "));
	Serial.println(F("--------------------------"));
	Serial.print(F("Device Address : 0x")); Serial.println(Sensor.Address(), HEX);
	Serial.print(F("Mux Channel    : ")); Serial.println(Sensor.Mux_Channel());
	Serial.print(F("Device Found   : ")); Serial.println((Sensor.Detect() ? F("OK") : F("FAIL")));
	Serial.println(F("--------------------------"));

	// Serial Print Data
	Serial.print(F("Delta Pressure : ")); Serial.print(Pressure, 4); Serial.println(F(" Pa"));
	Serial.print(F("Function Time  : ")); Serial.print(DT); Serial.println(F(" mS"));
	Serial.println(F("--------------------------"));

	// Loop Delay
	delay(1000);

}

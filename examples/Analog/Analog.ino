#include "Environment.h"

// Set Object
Analog Pressure(0x01, 10, true, 1.5777, -1.1925);

void setup() {

	// Serial Communication Start
	Serial.begin(115200);

	// Start I2C
	Wire.begin();

	// Header
	Serial.println("    Analog Measurement    ");
	Serial.println("--------------------------");

}

void loop() {

	// Set Start Time
	unsigned long Time = millis();

	// Measure
	float _Measurement = Pressure.Read();

	// Calculate Delta Time
	long DT = millis() - Time;

	// Serial Print Data
	Serial.print("Pressure      : "); Serial.print(_Measurement, 3); Serial.println(" Bar");
	Serial.print("Deviation     : "); Serial.print(Pressure.Standard_Deviation, 3); Serial.println("");
	Serial.print("Function Time : "); Serial.print(DT); Serial.println(" mS");
	Serial.println("--------------------------");

	// Loop Delay
	delay(1000);

}

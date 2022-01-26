#include "Environment.h"

void setup() {

	// Serial Communication Start
	Serial.begin(115200);

	// Start I2C
	Wire.begin();

	// Header
	Serial.println("      TSL2561 Light       ");
	Serial.println("--------------------------");

}

void loop() {

	// Set Start Time
	uint32_t Time = millis();

	// Measure
	float _Measurement = Sensor.TSL2561_Light();

	// Calculate Delta Time
	uint32_t DT = millis() - Time;

	// Serial Print Data
	Serial.print("Value         : ");
	Serial.print(_Measurement, 3);
	Serial.println(" Lux");
	Serial.print("Function Time : ");
	Serial.print(DT);
	Serial.println(" mS");
	Serial.println("--------------------------");

	// Loop Delay
	delay(1000);

}

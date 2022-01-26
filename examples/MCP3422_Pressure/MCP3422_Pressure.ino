#include "Environment.h"

void setup() {

	// Serial Communication Start
	Serial.begin(115200);

	// Start I2C
	Wire.begin();

	// Header
	Serial.println("     MCP3422 Pressure     ");
	Serial.println("--------------------------");

}

void loop() {

	// Set Start Time
	uint32_t Time = millis();

	// Measure
	float _Measurement = Sensor.MCP3422_Pressure(1, 10, 1);

	// Calculate Delta Time
	uint32_t DT = millis() - Time;

	// Serial Print Data
	Serial.print("Value         : ");
	Serial.print(_Measurement, 3);
	Serial.println(" Bar");
	Serial.print("Function Time : ");
	Serial.print(DT);
	Serial.println(" mS");
	Serial.println("--------------------------");

	// Loop Delay
	delay(1000);
	
}

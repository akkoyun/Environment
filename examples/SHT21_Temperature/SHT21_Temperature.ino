#include "Environment.h"

// Measure Parameters
int Sensor_Read_Count = 20;
int Sensor_Average_Type = 1;

// Define Library
Environment Environment;

void setup() {
	
	// Serial Communication Start
	Serial.begin(115200);
	
	// Start I2C
	Wire.begin();
	
}

void loop() {
	
	// Define Sensor Variables
	float _Measurement;
	float _Deviation;
	unsigned long Time;

	// Set Start Time
	Time = millis();

	// Measure
	_Measurement = Environment.SHT21_Temperature(Sensor_Read_Count, Sensor_Average_Type);

	// Calculate Delta Time
	int DT = millis() - Time;
	
	// Sensor EN to LOW
	pinMode(13, OUTPUT); digitalWrite(13, LOW);
	
	// Serial Print Data
	Serial.print("Value                   : "); Serial.print(_Measurement, 3); Serial.println(" C");
	Serial.print("Read Count              : "); Serial.println(Sensor_Read_Count);
	Serial.print("Function Time           : "); Serial.print(DT); Serial.println(" mS");
	Serial.println("--------------------------");
	
	// Loop Delay
	delay(1000);

}

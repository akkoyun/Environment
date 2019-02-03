#include "Noxcorp_Library.h"

// Measure Parameters
int Sensor_Read_Count = 5;
int Sensor_Average_Type = 5;

// Define Library
Noxcorp_Library* Noxcorp = 0;

void setup() {
	
	// Serial Communication Start
	Serial.begin(115200);
	
	// Start I2C
	Wire.begin();
	
	// Sensor EN to LOW
	pinMode(13, OUTPUT);
	digitalWrite(13, LOW);

}

void loop() {
	
	// Define Sensor Variables
	float _Air_Pressure = 0;
	double _Air_Pressure_Deviation = 0;
	unsigned long Time;

	// Sensor EN to HIGH
	pinMode(13, OUTPUT);
	digitalWrite(13, HIGH);
	delay(10);
	
	// Measure Pressure
	Time = millis();
	Noxcorp->B505AA_P(Sensor_Read_Count, Sensor_Average_Type, _Air_Pressure, _Air_Pressure_Deviation);
	int DT = millis() - Time;
	
	// Sensor EN to LOW
	pinMode(13, OUTPUT);
	digitalWrite(13, LOW);
	
	// Serial Print Data
	Serial.print("Value                   : "); Serial.print(_Air_Pressure, 5); Serial.println(" mBar");
	Serial.print("Standart Deviation      : "); Serial.println(_Air_Pressure_Deviation, 5);
	Serial.print("Read Count              : "); Serial.println(Sensor_Read_Count);
	Serial.print("Average Type            : "); Serial.println(Sensor_Average_Type);
	Serial.print("Function Time           : "); Serial.print(DT); Serial.println(" mS");
	Serial.println("------------------------------");
	
	// Loop Delay
	delay(1000);

}

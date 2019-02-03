#include "Noxcorp_Library.h"

// Measure Parameters
int Sensor_Read_Count = 50;
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
	float _Light_Lux = 0;
	double _Light_Lux_Deviation = 0;
	unsigned long Time;

	// Sensor EN to HIGH
	pinMode(13, OUTPUT);
	digitalWrite(13, HIGH);
	delay(10);
	
	// Measure Light
	Time = millis();
	Noxcorp->B501BA_L(Sensor_Read_Count, Sensor_Average_Type, _Light_Lux, _Light_Lux_Deviation);
	int DT = millis() - Time;
	
	// Sensor EN to LOW
	pinMode(13, OUTPUT);
	digitalWrite(13, LOW);
	
	// Serial Print Data
	Serial.print("Value                   : "); Serial.print(_Light_Lux, 5); Serial.println(" LUX");
	Serial.print("Standart Deviation      : "); Serial.println(_Light_Lux_Deviation, 5);
	Serial.print("Read Count              : "); Serial.println(Sensor_Read_Count);
	Serial.print("Average Type            : "); Serial.println(Sensor_Average_Type);
	Serial.print("Function Time           : "); Serial.print(DT); Serial.println(" mS");
	Serial.println("------------------------------");
	
	// Loop Delay
	delay(1000);

}

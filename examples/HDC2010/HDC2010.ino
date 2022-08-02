#include <Environment.h>
#include <Console.h>

// Define Object
Console Terminal(Serial);

void setup() {

	// Start Serial Stream
    Serial.begin(115200);

	// Start Terminal
    Terminal.Begin();

	// Draw Terminal Base
	Terminal.HDC2010_TH_Meter();

}

void loop() {

	// Define Sensor Object
	HDC2010 _Sensor(true, 3, 10, true);

	// Set Start Time
	unsigned long Time = millis();

	// Measure
	float Temperature = _Sensor.Temperature();
	float Humidity = _Sensor.Humidity();

	// Calculate Delta Time
	long DT = millis() - Time;

	// Print Terminal Text
	Terminal.Text(3, 26, CYAN, String(Temperature, 4));
	Terminal.Text(4, 26, CYAN, String(Humidity, 4));
	Terminal.Text(7, 30, CYAN, String(DT));

}
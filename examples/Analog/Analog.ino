#include <Environment.h>
#include <Console.h>

// Define Object
Console Terminal(Serial);

// Set Object
Analog Pressure(0x01, 10, true, 1.5777, -1.1925);

void setup() {

	// Start Serial Stream
    Serial.begin(115200);

	// Start Terminal
    Terminal.Begin();

	// Draw Terminal Base
	Terminal.Analog_Pressure_Meter();
}

void loop() {

	// Set Start Time
	unsigned long Time = millis();

	// Measure
	float _Measurement = Pressure.Read();

	// Calculate Delta Time
	long DT = millis() - Time;

	// Print Terminal Text
	Terminal.Text(3, 25, CYAN, String(_Measurement, 4));
	Terminal.Text(4, 29, CYAN, String(Pressure.Standard_Deviation, 4));
	Terminal.Text(7, 30, CYAN, String(DT));

}

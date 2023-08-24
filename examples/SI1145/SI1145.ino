#include <Environment.h>

// Define Sensor Object
SI1145 Sensor(true, __B108AA_MUX_SI1145__);

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
    float UV_Index = Sensor.Read_UV();
    float Visible = Sensor.Read_Visible();
    float IR = Sensor.Read_IR();

	// Calculate Delta Time
	long DT = millis() - Time;

	// Header
	Serial.println(F("      SI1145 L Sensor     "));
	Serial.println(F("--------------------------"));
	Serial.print(F("Device Address : 0x")); Serial.println(Sensor.Address(), HEX);
	Serial.print(F("Mux Channel    : ")); Serial.println(Sensor.Mux_Channel());
	Serial.print(F("Device Found   : ")); Serial.println((Sensor.Detect() ? F("OK") : F("FAIL")));
	Serial.println(F("--------------------------"));

	// Serial Print Data
	Serial.print(F("Visible Light  : ")); Serial.print(Visible, 2); Serial.println(F(""));
	Serial.print(F("IR Light       : ")); Serial.print(IR, 2); Serial.println(F(""));
	Serial.print(F("UV Index       : ")); Serial.print(UV_Index, 2); Serial.println(F(""));
	Serial.print(F("Function Time  : ")); Serial.print(DT); Serial.println(F(" mS"));
	Serial.println(F("--------------------------"));

	// Loop Delay
	delay(1000);

}

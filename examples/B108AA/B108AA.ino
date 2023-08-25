// Include Sensor Library
#include <Environment.h>

// Define Sensor Object
B108AA_Environment Environment;

void setup() {

	// Serial Communication Start
	Serial.begin(115200);

	// Start I2C Communication
    Environment.Start();

}

void loop() {

	// Header
	Serial.println(F("       Environment        "));
	Serial.println(F("--------------------------"));

	// Serial Print Data
	Serial.print(F("Temperature    : ")); Serial.print(Environment.Temperature(), 2); Serial.println(F(" C"));
	Serial.print(F("Humidity       : ")); Serial.print(Environment.Humidity(), 2); Serial.println(F(" %"));
    Serial.print(F("Pressure       : ")); Serial.print(Environment.Pressure(), 2); Serial.println(F(" hPa"));
  	Serial.print(F("Visible Light  : ")); Serial.print(Environment.Visible(), 2); Serial.println(F(""));
	Serial.print(F("IR Light       : ")); Serial.print(Environment.IR(), 2); Serial.println(F(""));
	Serial.print(F("UV Index       : ")); Serial.print(Environment.UV(), 2); Serial.println(F(""));

    // Declare Variables
    float _Wind_Speed = 0;
    float _Wind_Direction = 0;

    // Get Wind Speed
    Environment.Wind(_Wind_Direction, _Wind_Speed);

	Serial.print(F("Wind Direction : ")); Serial.print(_Wind_Direction, 2); Serial.println(F(""));
	Serial.print(F("Wind Speed     : ")); Serial.print(_Wind_Speed, 2); Serial.println(F(""));

	Serial.println(F("--------------------------"));

	// Loop Delay
	delay(1000);

}

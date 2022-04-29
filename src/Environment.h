/* *******************************************************************************
 *  Copyright (C) 2014-2019 Mehmet Gunce Akkoyun Can not be copied and/or
 *	distributed without the express permission of Mehmet Gunce Akkoyun
 *	This library is a combined book of enviroment sensor library.
 *
 *	Library				: Environment Library.
 *	Code Developer		: Mehmet Gunce Akkoyun (akkoyun@me.com)
 *********************************************************************************/

#ifndef __Environment__
#define __Environment__

// Define Arduino Library
#ifndef __Arduino__
#include <Arduino.h>
#endif

// Define I2C Functions Library
#ifndef __I2C_Functions__
#include <I2C_Functions.h>
#endif

// Define Statistical Library
#ifndef __Statistical__
#include <Statistical.h>
#endif

// HDC2010 Class
class HDC2010 {

	private:

		// Read Count
		uint8_t _Read_Count = 10;

		// Calibration Parameters
		bool _Calibration = false;

	public:

		// Constractor
		HDC2010(uint8_t _Measurement_Count, bool _Calibration_Enable);

		// Temperature
		float Temperature(void);

		// Humidity
		float Humidity(void);

};

// SHT21 Class
class SHT21 {

	private:

		// Read Count
		uint8_t _Read_Count = 10;

		// Calibration Parameters
		bool _Calibration = true;

	public:

		// Constractor
		SHT21(uint8_t _Measurement_Count, bool _Calibration_Enable);

		// Temperature
		float Temperature(void);

		// Humidity
		float Humidity(void);

};

// MPL3115A2 Class
class MPL3115A2 {

	private:

		// Read Count
		uint8_t _Read_Count = 10;

		// Calibration Parameters
		bool _Calibration = true;

	public:

		// Constractor
		MPL3115A2(uint8_t _Measurement_Count, bool _Calibration_Enable);

		// Pressure
		float Pressure(void);

};

// TSL2561 Class
class TSL2561 {

	private:

		// Read Count
		uint8_t _Read_Count = 10;

		// Calibration Parameters
		bool _Calibration = true;

	public:

		// Constractor
		TSL2561(uint8_t _Measurement_Count, bool _Calibration_Enable);

		// Light
		float Light(void);

};

// MCP3422 Class
class MCP3422 {

	private:

		// Read Count
		uint8_t _Read_Count = 10;

		// Channel
		uint8_t _Channel = 1;

		// Calibration Parameters
		bool _Calibration = true;

	public:

		// Constractor
		MCP3422(uint8_t _Channel, uint8_t _Measurement_Count, bool _Calibration_Enable);

		// Light
		float Pressure(void);

};

// Analog Read Class
class Analog {

	private:

		// Analog Class Struct Defination
		struct Analog_Struct {

			// Read Count
			uint8_t Read_Count;

			// Calibration Parameters
			bool Calibration;

			// Calibration Parameters
			float Cal_a;
			float Cal_b;

		} Measurement;

	public:

		// Statistical Parameeters
		float Standart_Deviation;

		// Constractor
		Analog(uint8_t _Channel, uint8_t _Read_Count, bool _Calibration, float _Cal_a, float _Cal_b);

		// Read
		double Read(void);

};

#endif /* defined(__Environment__) */

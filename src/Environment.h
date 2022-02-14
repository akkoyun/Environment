/* *******************************************************************************
 *  Copyright (C) 2014-2019 Mehmet Gunce Akkoyun Can not be copied and/or
 *	distributed without the express permission of Mehmet Gunce Akkoyun
 *	This library is a combined book of enviroment sensor library.
 *
 *	Library				: Environment Library.
 *	Code Developer		: Mehmet Gunce Akkoyun (akkoyun@me.com)
 *	Revision			: 3.1.0
 *	Relase				: 12.10.2020
 *
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

class Environment {

	public:
		
		// ************************************************************
		// Public Functions
		// ************************************************************

		// SHT21
		float SHT21_Temperature(const uint8_t Read_Count_, const uint8_t Average_Type_);
		float SHT21_Humidity(const uint8_t Read_Count_, const uint8_t Average_Type_);

		// HDC2010
		float HDC2010_Temperature(const uint8_t Read_Count_, const uint8_t Average_Type_);
		float HDC2010_Humidity(const uint8_t Read_Count_, const uint8_t Average_Type_);

		// MPL3115A2
		float MPL3115A2_Pressure(void);

		// TSL2561
		float TSL2561_Light(void);
		
		// MCP3422
		float MCP3422_Pressure(const uint8_t _Channel, const uint8_t _Read_Count, const uint8_t _Average_Type);

	private:

		// ************************************************************
		// Calibration Constants
		// ************************************************************

		// SHT21 Temperature
		const float SHT21_T_Calibrarion_a = 1.0129; // MGM
		const float SHT21_T_Calibrarion_b = 0.6075; // MGM

		// SHT21 Humidity
		const float SHT21_H_Calibrarion_a = 0.9518; // MGM
		const float SHT21_H_Calibrarion_b = 3.5316; // MGM

		// MPL3115A2 Pressure
		const float MPL3115A2_P_Calibrarion_a = 1;
		const float MPL3115A2_P_Calibrarion_b = 0;

		// HDC2010 Temperature
		const float HDC2010_T_Calibrarion_a = 1.0053;
		const float HDC2010_T_Calibrarion_b = -0.4102;
		
		// HDC2010 Humidity
		const float HDC2010_H_Calibrarion_a = 0.9821;
		const float HDC2010_H_Calibrarion_b = -0.3217;

		// MCP3422 Humidity
		const float MCP3422_P_Calibrarion_a = 1; //1.5304
		const float MCP3422_P_Calibrarion_b = 0; // -1,3437

};

extern Environment Sensor;

#endif /* defined(__Environment__) */

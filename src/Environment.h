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

#include <Arduino.h>
#include <Wire.h>

//#define Environment_Detail

class Environment
{
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
	const float HDC2010_T_Calibrarion_a = 1;
	const float HDC2010_T_Calibrarion_b = 0;
	
	// HDC2010 Humidity
	const float HDC2010_H_Calibrarion_a = 1;
	const float HDC2010_H_Calibrarion_b = 0;

};

#endif /* defined(__Environment__) */

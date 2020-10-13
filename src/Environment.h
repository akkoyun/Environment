/* *******************************************************************************
 *  Copyright (C) 2014-2019 Mehmet Gunce Akkoyun Can not be copied and/or
 *	distributed without the express permission of Mehmet Gunce Akkoyun
 *	This library is a combined book of enviroment sensor library.
 *
 *	Library				: Environment Library.
 *	Code Developer		: Mehmet Gunce Akkoyun (akkoyun@me.com)
 *	Revision			: 3.0.0
 *	Relase				: 12.10.2020
 *
 *********************************************************************************/

#ifndef __Environment__
#define __Environment__

#include <Arduino.h>
#include <Wire.h>

class Environment
{
public:

	// Define Function Versions
	const PROGMEM char SHT21_T_Version[9] 		= "03.13.02";
	const PROGMEM char SHT21_H_Version[9] 		= "03.13.02";
	const PROGMEM char MPL3115A2_P_Version[9]	= "03.05.00";
	const PROGMEM char TSL2561_L_Version[9] 	= "03.03.00";
	const PROGMEM char HDC2010_T_Version[9] 	= "01.00.00";
	const PROGMEM char HDC2010_H_Version[9] 	= "01.00.00";

	// SHT21
	bool SHT21_T(uint8_t Read_Count, uint8_t AVG_Type, float &Value_, float &Deviation_);		// SHT21 Temperature
	bool SHT21_H(uint8_t Read_Count, uint8_t AVG_Type, float &Value_, float &Deviation_);		// SHT21 Humidity
	
	// MPL3115A2
	bool MPL3115A2_P(uint8_t Read_Count, uint8_t AVG_Type, float &Value_, float &Deviation_);	// MPL3115A2 Pressure
	
	// TSL2561
	bool TSL2561_L(uint8_t Read_Count, uint8_t AVG_Type, float &Value_, float &Deviation_);		// TSL2561FN Light

	// HDC2010
	bool HDC2010_T(uint8_t Read_Count, uint8_t AVG_Type, float &Value_, float &Deviation_); 	// HDC2010 Temperature
	bool HDC2010_H(uint8_t Read_Count, uint8_t AVG_Type, float &Value_, float &Deviation_); 	// HDC2010 Humidity

private:

	// Calibration Constants
	float SHT21_T_Calibrarion_a = 1.0129; // MGM
	float SHT21_T_Calibrarion_b = 0.6075; // MGM

	float SHT21_H_Calibrarion_a = 0.9518; // MGM
	float SHT21_H_Calibrarion_b = 3.5316; // MGM

	float MPL3115A2_P_Calibrarion_a = 1;
	float MPL3115A2_P_Calibrarion_b = 0;

	float HDC2010_T_Calibrarion_a = 1;
	float HDC2010_T_Calibrarion_b = 0;
	float HDC2010_H_Calibrarion_a = 1;
	float HDC2010_H_Calibrarion_b = 0;

};

#endif /* defined(__Environment__) */

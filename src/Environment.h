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

	// ************************************************************
	// Public Variables
	// ************************************************************


	// ************************************************************
	// Public Functions
	// ************************************************************
	float Read_Sensor(uint8_t Sensor_ID_, uint8_t Read_Count_, uint8_t Average_Type);

private:

	// ************************************************************
	// Private Functions
	// ************************************************************

	// Sensirion SHT21
	bool SHT21_Temperature(float &Value_);
	bool SHT21_Humidity(float &Value_);

	// MPL3115A2
	bool MPL3115A2_Pressure(float &Value_);

	// HDC2010
	bool HDC2010_Temperature(float &Value_);
	bool HDC2010_Humidity(float &Value_);

	// TSL2561
	bool TSL2561_Light(float &Value_);

	// ************************************************************
	// Private Variables
	// ************************************************************

	uint8_t Sensor_Count = 2;
	
	// const char SHT21_Temperature_Version[9] 	= "04.00.00";

	// ************************************************************
	// Calibration Constants
	// ************************************************************

	// SHT21 Temperature
	float SHT21_T_Calibrarion_a = 1.0129; // MGM
	float SHT21_T_Calibrarion_b = 0.6075; // MGM

	// SHT21 Humidity
	float SHT21_H_Calibrarion_a = 0.9518; // MGM
	float SHT21_H_Calibrarion_b = 3.5316; // MGM

	// MPL3115A2 Pressure
	float MPL3115A2_P_Calibrarion_a = 1;
	float MPL3115A2_P_Calibrarion_b = 0;

	// HDC2010 Temperature
	float HDC2010_T_Calibrarion_a = 1;
	float HDC2010_T_Calibrarion_b = 0;
	
	// HDC2010 Humidity
	float HDC2010_H_Calibrarion_a = 1;
	float HDC2010_H_Calibrarion_b = 0;

};

#endif /* defined(__Environment__) */

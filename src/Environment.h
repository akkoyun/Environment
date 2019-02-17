/* *******************************************************************************
 *  Copyright (C) 2014-2019 Mehmet Gunce Akkoyun Can not be copied and/or
 *	distributed without the express permission of Mehmet Gunce Akkoyun
 *	This library is a combined book of enviroment sensor library.
 *
 *	Library				: Environment Library.
 *	Code Developer		: Mehmet Gunce Akkoyun (akkoyun@me.com)
 *	Revision			: 2.0.0
 *	Relase				: 17.02.2019
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
	String B502BA_T_Version = "03.13.02";
	String B502BA_H_Version = "03.13.02";
	String B505AA_P_Version = "03.05.00";
	String B501BA_L_Version = "03.03.00";

	// Environment
	bool B502BA_T(int Read_Count, int AVG_Type, float &Value_, double &Deviation_);	// SHT21
	bool B502BA_H(int Read_Count, int AVG_Type, float &Value_, double &Deviation_);	// SHT21
	bool B505AA_P(int Read_Count, int AVG_Type, float &Value_, double &Deviation_);	// MPL3115A2
	bool B501BA_L(int Read_Count, int AVG_Type, float &Value_, double &Deviation_);	// TSL2561FN

private:

	// Calibration Constants
	float B502BA_T_Calibrarion_a = 1.0129; // MGM
	float B502BA_T_Calibrarion_b = 0.6075; // MGM
	float B502BA_H_Calibrarion_a = 0.9518; // MGM
	float B502BA_H_Calibrarion_b = 3.5316; // MGM

};

#endif /* defined(__Environment__) */

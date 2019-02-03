/* *******************************************************************************
 *  Copyright (C) 2014-2019 Noxcorp Elektronik A.S. (info@noxcorp.org)
 *  Can not be copied and/or distributed without the express permission of Noxcorp
 *	This library is a combined book of Noxcorp enviroment sensor library.
 *
 *	The library is compatable with Noxcorp Hardware Community.
 *
 *	Library				: Noxcorp Enviroment Library.
 *	Code Developer		: Mehmet Gunce Akkoyun (gunce.akkoyun@noxcorp.org)
 *	Revision			: 1.0.0
 *	Relase				: 03.02.2019
 *
 *********************************************************************************/

#ifndef __Nox_Enviroment__
#define __Nox_Enviroment__

#include <Arduino.h>
#include <Wire.h>

class Nox_Enviroment
{
public:

	// Define Function Versions
	String B502BA_T_Version = "03.13.00";
	String B502BA_H_Version = "03.13.00";
	String B505AA_P_Version = "03.05.00";
	String B501BA_L_Version = "03.03.00";

	// Enviroment
	bool B502BA_T(int Read_Count, int AVG_Type, float &Value_, double &Deviation_);	// SHT21
	bool B502BA_H(int Read_Count, int AVG_Type, float &Value_, double &Deviation_);	// SHT21
	bool B505AA_P(int Read_Count, int AVG_Type, float &Value_, double &Deviation_);	// MPL3115A2
	bool B501BA_L(int Read_Count, int AVG_Type, float &Value_, double &Deviation_);	// TSL2561FN

private:

};

#endif /* defined(__Nox_Enviroment__) */

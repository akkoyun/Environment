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

#include "Arduino.h"
#include "Environment.h"
#include <Wire.h>

// Environment
bool Environment::SHT21_Temperature(float &Value_) {
	
	/******************************************************************************
	 *	Project		: SHT21 Temperature Read Function
	 *	Developer	: Mehmet Gunce Akkoyun (akkoyun@me.com)
	 *	Revision	: 04.00.00
	 *	Release		: 04.11.2020
	 ******************************************************************************/
	
	// Set Sensor Definations
	struct Sensor_Settings {
		int 	Resolution;
		bool	EoB;
		bool	OCH;
		bool	OTP;
		int		Range_Min;
		int		Range_Max;
		
	};
	Sensor_Settings SHT21[] {
		
		{
			14,			// Measurement Resolution
			false,		// Sensor End of Battery Setting
			false,		// On Chip Heater Setting
			true,		// OTP Read
			-40,		// Sensor Range Minimum
			100			// Sensor Range Maximum
			
		}
		
	};
	
	// ************************************************************
	// Set Sensor Configuration Byte
	// ************************************************************
	
	// User Register Bit Definations
	bool User_Reg_Bits_[8] = {false, false, false, false, false, false, false, false};
	
	// Set Resolution Bits
	if (SHT21[0].Resolution == 14)	{ User_Reg_Bits_[7] = false	; User_Reg_Bits_[0] = false	;} // T: 14 bit
	if (SHT21[0].Resolution == 12)	{ User_Reg_Bits_[7] = false	; User_Reg_Bits_[0] = true	;} // T: 12 bit
	if (SHT21[0].Resolution == 13)	{ User_Reg_Bits_[7] = true	; User_Reg_Bits_[0] = false	;} // T: 13 bit
	if (SHT21[0].Resolution == 11)	{ User_Reg_Bits_[7] = true	; User_Reg_Bits_[0] = true	;} // T: 11 bit
	
	// Set End of Battery Bits
	if (SHT21[0].EoB == true) {User_Reg_Bits_[6] = true;} else {User_Reg_Bits_[6] = false;}
	
	// On Chip Heater Bits
	if (SHT21[0].OCH == true) {User_Reg_Bits_[2] = true;} else {User_Reg_Bits_[2] = false;}
	
	// OTP Reload Bits
	if (SHT21[0].OTP == true) {User_Reg_Bits_[1] = true;} else {User_Reg_Bits_[1] = false;}
	
	// User Register Defination
	uint8_t User_Reg_ = 0x00;
	
	// Set Config Register
	if (User_Reg_Bits_[0] == true) {User_Reg_ |= 0b00000001;} else {User_Reg_ &= 0b11111110;}	// User Register Bit 0
	if (User_Reg_Bits_[1] == true) {User_Reg_ |= 0b00000010;} else {User_Reg_ &= 0b11111101;}	// User Register Bit 1
	if (User_Reg_Bits_[2] == true) {User_Reg_ |= 0b00000100;} else {User_Reg_ &= 0b11111011;}	// User Register Bit 2
	if (User_Reg_Bits_[3] == true) {User_Reg_ |= 0b00001000;} else {User_Reg_ &= 0b11110111;}	// User Register Bit 3
	if (User_Reg_Bits_[4] == true) {User_Reg_ |= 0b00010000;} else {User_Reg_ &= 0b11101111;}	// User Register Bit 4
	if (User_Reg_Bits_[5] == true) {User_Reg_ |= 0b00100000;} else {User_Reg_ &= 0b11011111;}	// User Register Bit 5
	if (User_Reg_Bits_[6] == true) {User_Reg_ |= 0b01000000;} else {User_Reg_ &= 0b10111111;}	// User Register Bit 6
	if (User_Reg_Bits_[7] == true) {User_Reg_ |= 0b10000000;} else {User_Reg_ &= 0b01111111;}	// User Register Bit 7
	
	// ************************************************************
	// Reset Sensor
	// ************************************************************
	
	// Send Soft Reset Command to SHT21
	Wire.beginTransmission(0b01000000);
	Wire.write(0b11111110);
	
	// Close I2C Connection
	int SHT21_Reset = Wire.endTransmission(false);
	
	// Control For Reset Success
	if (SHT21_Reset != 0) {
		
		// Set Error Code
		Value_ = -101;
		
		// End Function
		return(false);
		
	}
	
	// Software Reset Delay
	delay(15);
	
	// ************************************************************
	// Read Current Sensor Settings
	// ************************************************************
	
	// Read User Register of SHT21
	Wire.beginTransmission(0b01000000);
	Wire.write(0b11100110);
	Wire.requestFrom(0b01000000, 1);
	uint8_t SHT21_Config_Read = Wire.read();
	
	// ************************************************************
	// Write New Settings if Different
	// ************************************************************
	
	// Control for Config Read Register
	if (SHT21_Config_Read != User_Reg_) {
		
		// Write User Register of SHT21
		Wire.beginTransmission(0b01000000);
		Wire.write(0b11100110);
		Wire.write(User_Reg_);
		
		// Close I2C Connection
		int SHT21_Config = Wire.endTransmission(false);
		
		// Control For Command Success
		if (SHT21_Config != 0) {
			
			// Set Error Code
			Value_ = -102;
			
			// End Function
			return(false);
			
		}
		
	}
	
	// ************************************************************
	// Read Sensor Data
	// ************************************************************
	
	// Define Variables
	uint16_t Measurement_Raw = 0;
	
	// Send Read Command to SHT21
	Wire.beginTransmission(0b01000000);
	Wire.write(0b11100011);
	
	// Close I2C Connection
	int SHT21_Read = Wire.endTransmission(false);
	
	// Control For Read Success
	if (SHT21_Read != 0) {
		
		// Set Error Code
		Value_ = -103;
		
		// End Function
		return(false);
		
	}
	
	// Read Data Command to SHT21
	Wire.requestFrom(0b01000000, 3);
	
	// Define Data Variable
	uint8_t SHT21_Data[3];
	
	// Read I2C Bytes
	SHT21_Data[0] = Wire.read(); // MSB
	SHT21_Data[1] = Wire.read(); // LSB
	SHT21_Data[2] = Wire.read(); // CRC
	
	// Combine Read Bytes
	Measurement_Raw = ((SHT21_Data[0] << 8) | SHT21_Data[1]);
	
	// Clear 2 Low Status Bit
	if (SHT21[0].Resolution == 11) Measurement_Raw &= ~0x0006;
	if (SHT21[0].Resolution == 12) Measurement_Raw &= ~0x0005;
	if (SHT21[0].Resolution == 13) Measurement_Raw &= ~0x0004;
	if (SHT21[0].Resolution == 14) Measurement_Raw &= ~0x0003;
	
	// Define CRC Variables
	uint8_t CRC, Control_Byte;
	
	// Calculate CRC
	for (Control_Byte = 0; Control_Byte < 3; Control_Byte++) {
		CRC ^= SHT21_Data[Control_Byte];
		for (uint8_t bit = 8; bit > 0; bit--) {
			if (CRC & 0x80) {CRC = (CRC << 1) ^ 0x131;}
			else { CRC = (CRC << 1);}
		}
	}
	
	// Control and Calculate Measurement
	if (CRC == 0) {
		
		// Calculate Measurement
		Value_ = (SHT21_T_Calibrarion_a * (-46.85 + ((175.72 * float(Measurement_Raw)) / pow(2,16)))) + SHT21_T_Calibrarion_b;
		
	} else {
		
		// Set Error Code
		Value_ = -104;
		
		// End Function
		return(false);
		
	}
	
	// Read Delay
	if (SHT21[0].Resolution == 14) delay(85); 	// T: 14 bit
	if (SHT21[0].Resolution == 12) delay(22);	// T: 12 bit
	if (SHT21[0].Resolution == 13) delay(43); 	// T: 13 bit
	if (SHT21[0].Resolution == 11) delay(11);	// T: 11 bit
	
	// ************************************************************
	// Control For Sensor Range
	// ************************************************************
	if (Value_ < SHT21[0].Range_Min or Value_ > SHT21[0].Range_Max) {
		
		// Set Error Code
		Value_ = -106;
		
		// End Function
		return(false);
		
	}
	
	// End Function
	return(true);

}
bool Environment::SHT21_Humidity(float &Value_) {
	
	/******************************************************************************
	 *	Project		: SHT21 Humidity Read Function
	 *	Developer	: Mehmet Gunce Akkoyun (akkoyun@me.com)
	 *	Revision	: 04.00.00
	 *	Release		: 04.11.2020
	 ******************************************************************************/
	
	// Set Sensor Definations
	struct Sensor_Settings {
		int 	Resolution;
		bool	EoB;
		bool	OCH;
		bool	OTP;
		int		Range_Min;
		int		Range_Max;
		
	};
	Sensor_Settings SHT21[] {
		
		{
			12,			// Measurement Resolution
			false,		// Sensor End of Battery Setting
			false,		// On Chip Heater Setting
			false,		// OTP Read
			0,			// Sensor Range Minimum
			100			// Sensor Range Maximum
			
		}
		
	};
	
	// ************************************************************
	// Set Sensor Configuration Byte
	// ************************************************************
	
	// User Register Bit Definations
	bool User_Reg_Bits_[8] = {false, false, false, false, false, false, false, false};
	
	// Set Resolution Bits
	if (SHT21[0].Resolution == 12)	{ User_Reg_Bits_[7] = false	; User_Reg_Bits_[0] = false	;} // T: 12 bit
	if (SHT21[0].Resolution == 11)	{ User_Reg_Bits_[7] = true	; User_Reg_Bits_[0] = true	;} // T: 11 bit
	if (SHT21[0].Resolution == 10)	{ User_Reg_Bits_[7] = true	; User_Reg_Bits_[0] = false	;} // T: 10 bit
	if (SHT21[0].Resolution == 8)	{ User_Reg_Bits_[7] = false	; User_Reg_Bits_[0] = true	;} // T: 08 bit
	
	// Set End of Battery Bits
	if (SHT21[0].EoB == true) {User_Reg_Bits_[6] = true;} else {User_Reg_Bits_[6] = false;}
	
	// On Chip Heater Bits
	if (SHT21[0].OCH == true) {User_Reg_Bits_[2] = true;} else {User_Reg_Bits_[2] = false;}
	
	// OTP Reload Bits
	if (SHT21[0].OTP == true) {User_Reg_Bits_[1] = true;} else {User_Reg_Bits_[1] = false;}
	
	// User Register Defination
	uint8_t User_Reg_ = 0x00;
	
	// Set Config Register
	if (User_Reg_Bits_[0] == true) {User_Reg_ |= 0b00000001;} else {User_Reg_ &= 0b11111110;}	// User Register Bit 0
	if (User_Reg_Bits_[1] == true) {User_Reg_ |= 0b00000010;} else {User_Reg_ &= 0b11111101;}	// User Register Bit 1
	if (User_Reg_Bits_[2] == true) {User_Reg_ |= 0b00000100;} else {User_Reg_ &= 0b11111011;}	// User Register Bit 2
	if (User_Reg_Bits_[3] == true) {User_Reg_ |= 0b00001000;} else {User_Reg_ &= 0b11110111;}	// User Register Bit 3
	if (User_Reg_Bits_[4] == true) {User_Reg_ |= 0b00010000;} else {User_Reg_ &= 0b11101111;}	// User Register Bit 4
	if (User_Reg_Bits_[5] == true) {User_Reg_ |= 0b00100000;} else {User_Reg_ &= 0b11011111;}	// User Register Bit 5
	if (User_Reg_Bits_[6] == true) {User_Reg_ |= 0b01000000;} else {User_Reg_ &= 0b10111111;}	// User Register Bit 6
	if (User_Reg_Bits_[7] == true) {User_Reg_ |= 0b10000000;} else {User_Reg_ &= 0b01111111;}	// User Register Bit 7
	
	// ************************************************************
	// Reset Sensor
	// ************************************************************
	
	// Send Soft Reset Command to SHT21
	Wire.beginTransmission(0b01000000);
	Wire.write(0b11111110);
	
	// Close I2C Connection
	int SHT21_Reset = Wire.endTransmission(false);
	
	// Control For Reset Success
	if (SHT21_Reset != 0) {
		
		// Set Error Code
		Value_ = -101;
		
		// End Function
		return(false);
		
	}
	
	// Software Reset Delay
	delay(15);
	
	// ************************************************************
	// Read Current Sensor Settings
	// ************************************************************
	
	// Read User Register of SHT21
	Wire.beginTransmission(0b01000000);
	Wire.write(0b11100110);
	Wire.requestFrom(0b01000000, 1);
	uint8_t SHT21_Config_Read = Wire.read();
	
	// ************************************************************
	// Write New Settings if Different
	// ************************************************************
	
	// Control for Config Read Register
	if (SHT21_Config_Read != User_Reg_) {
		
		// Write User Register of SHT21
		Wire.beginTransmission(0b01000000);
		Wire.write(0b11100110);
		Wire.write(User_Reg_);
		
		// Close I2C Connection
		int SHT21_Config = Wire.endTransmission(false);
		
		// Control For Command Success
		if (SHT21_Config != 0) {
			
			// Set Error Code
			Value_ = -102;
			
			// End Function
			return(false);
			
		}
		
	}
	
	// ************************************************************
	// Read Sensor Datas
	// ************************************************************
	
	// Define Variables
	uint16_t Measurement_Raw = 0;
	
	// Send Read Command to SHT21
	Wire.beginTransmission(0b01000000);
	Wire.write(0b11100101);
	
	// Close I2C Connection
	int SHT21_Read = Wire.endTransmission(false);
	
	// Control For Read Success
	if (SHT21_Read != 0) {
		
		// Set Error Code
		Value_ = -103;
		
		// End Function
		return(false);
		
	}
	
	// Read Data Command to SHT21
	Wire.requestFrom(0b01000000, 3);
	
	// Define Data Variable
	uint8_t SHT21_Data[3];
	
	// Read I2C Bytes
	SHT21_Data[0] = Wire.read(); // MSB
	SHT21_Data[1] = Wire.read(); // LSB
	SHT21_Data[2] = Wire.read(); // CRC
	
	// Combine Read Bytes
	Measurement_Raw = ((SHT21_Data[0] << 8) | SHT21_Data[1]);
	
	// Define CRC Variables
	uint8_t CRC, Control_Byte;
	
	// Calculate CRC
	for (Control_Byte = 0; Control_Byte < 3; Control_Byte++) {
		CRC ^= SHT21_Data[Control_Byte];
		for (uint8_t bit = 8; bit > 0; bit--) {
			if (CRC & 0x80) {CRC = (CRC << 1) ^ 0x131;}
			else { CRC = (CRC << 1);}
		}
	}
	
	// Control and Calculate Measurement
	if (CRC == 0) {
		
		// Calculate Measurement
		Value_ = (SHT21_H_Calibrarion_a * (-6 + ((125 * float(Measurement_Raw)) / pow(2,16))));
		
	} else {
		
		// Set Error Code
		Value_ = -104;
		
		// End Function
		return(false);
		
	}
	
	// Read Delay
	if (SHT21[0].Resolution == 12) delay(29); 	// T: 12 bit
	if (SHT21[0].Resolution == 11) delay(15);	// T: 11 bit
	if (SHT21[0].Resolution == 10) delay(9); 	// T: 10 bit
	if (SHT21[0].Resolution == 8) delay(4);		// T: 08 bit
	
	// ************************************************************
	// Control For Sensor Range
	// ************************************************************
	if (Value_ < SHT21[0].Range_Min or Value_ > SHT21[0].Range_Max) {
		
		// Set Error Code
		Value_ = -106;
		
		// End Function
		return(false);
		
	}
	
	// End Function
	return(true);

}










bool Environment::MPL3115A2_P(uint8_t Read_Count, uint8_t AVG_Type, float &Value_, float &Deviation_) {

	/******************************************************************************
	 *	Project		: MPL3115A2 Pressure Read Function
	 *	Developer	: Mehmet Gunce Akkoyun (akkoyun@me.com)
	 *	Revision	: 03.06.00
	 *	Relase		: 25.12.2018
	 *	AVG Type	: 1-AVG, 2-RMS, 3-EXRMS, 4-MEDIAN, 5-Sigma1RMS
	 ******************************************************************************/

	// Set Sensor Definations
	struct Sensor_Settings {
		int		Range_Min;
		int		Range_Max;
		
	};
	Sensor_Settings MPL3115A2[] {
		
		500,		// Sensor Range Minimum
		11000,		// Sensor Range Maximum

	};

	// Control Read Count
	if (Read_Count < 1) Read_Count = 1;
	if (Read_Count > 50) Read_Count = 50;

	// Control for EXRMS Read Count
	if (Read_Count < 3 and AVG_Type == 3) AVG_Type = 2;

	// Define Pressure Read Array
	float Measurement_Array[Read_Count];

	// ************************************************************
	// Controll For WHO_AM_I Register
	// ************************************************************

	// Request WHO_AM_I Register
	Wire.beginTransmission(0b01100000);
	Wire.write(0b00001100);

	// Close I2C Connection
	int MPL3115A2_Sensor_Identification = Wire.endTransmission(false);

	// Control For Identifier Read Success
	if (MPL3115A2_Sensor_Identification != 0) {
		
		// Set Error Code
		Value_ = -101;
		
		// End Function
		return(false);
		
	}

	// Read Device Identifier Register
	Wire.requestFrom(0b01100000, 1);

	// Read WHO_AM_I Register
	uint8_t MPL3115A2_Device_Signiture = Wire.read();

	// ************************************************************
	// Control for Sensor ID
	// ************************************************************

	// Control for Device Identifier
	if (MPL3115A2_Device_Signiture == 0b11000100) {
		
		// ************************************************************
		// Set CTRL_REG1 Register
		// ************************************************************
		
		// Set CTRL_REG1 Register
		Wire.beginTransmission(0b01100000);
		Wire.write(0b00100110);
		Wire.write(0b00111001);
		
		// Close I2C Connection
		int MPL3115A2_Sensor_CTRL_REG1_Register = Wire.endTransmission(false);
		
		// Control For Register Write
		if (MPL3115A2_Sensor_CTRL_REG1_Register != 0) {
			
			// Set Error Code
			Value_ = -102;
			
			// End Function
			return(false);
			
		}
		
		// ************************************************************
		// Set PT_DATA_CFG Register
		// ************************************************************
		
		// Set PT_DATA_CFG Register
		Wire.beginTransmission(0b01100000);
		Wire.write(0b00010011);
		Wire.write(0b00000111);
		
		// Close I2C Connection
		int MPL3115A2_Sensor_PT_DATA_CFG_Register = Wire.endTransmission(false);
		
		// Control For Register Write
		if (MPL3115A2_Sensor_PT_DATA_CFG_Register != 0) {
			
			// Set Error Code
			Value_ = -103;
			
			// End Function
			return(false);
			
		}
		
		// ************************************************************
		// Read Sensor Datas
		// ************************************************************
		
		// Read Loop For Read Count
		for (int Read_ID = 0; Read_ID < Read_Count; Read_ID++) {
			
			// Define Variables
			uint8_t MPL3115A2_Read_Status = 0;
			uint8_t Ready_Status_Try_Counter = 0;
			
			// ************************************************************
			// Wait for Measurement Complate
			// ************************************************************
			while ((MPL3115A2_Read_Status & 0b00000100) != 0b00000100) {
				
				//  Request Pressure Ready Status
				Wire.beginTransmission(0b01100000);
				Wire.write(0b00000000);
				
				// Close I2C Connection
				int MPL3115A2_Sensor_Pressure_Ready_Status = Wire.endTransmission(false);
				
				// Control For Ready Status Write
				if (MPL3115A2_Sensor_Pressure_Ready_Status != 0) {
					
					// Set Error Code
					Value_ = -104;
					
					// End Function
					return(false);
					
				}
				
				// Read Device Status Register
				Wire.requestFrom(0b01100000, 1);
				MPL3115A2_Read_Status = Wire.read();
				
				// Increase Counter
				Ready_Status_Try_Counter += 1;
				
				// Control for Wait Counter
				if (Ready_Status_Try_Counter > 50) {
					
					// Set Error Code
					Value_ = -105;
					
					// End Function
					return(false);
					
				}
				
				// Ready Status Wait Delay
				if ((MPL3115A2_Read_Status & 0b00000100) != 0b00000100) delay(50);
				
			}
			
			// ************************************************************
			// Read Sensor Data
			// ************************************************************
			
			// Request Pressure Data
			Wire.beginTransmission(0b01100000);
			Wire.write(0b00000001);
			
			// Close I2C Connection
			int MPL3115A2_Sensor_Data_Read = Wire.endTransmission(false);
			
			// Control For Read Command Success
			if (MPL3115A2_Sensor_Data_Read != 0) {
				
				// Set Error Code
				Value_ = -106;
				
				// End Function
				return(false);
				
			}
			
			// Request Pressure Data
			Wire.requestFrom(0b01100000,3);
			
			// Define Data Variable
			uint8_t MPL3115A2_Data[3];
			
			// Read I2C Bytes
			MPL3115A2_Data[0] = Wire.read();
			MPL3115A2_Data[1] = Wire.read();
			MPL3115A2_Data[2] = Wire.read();
			
			// ************************************************************
			// Calculate Measurement Value
			// ************************************************************
			
			// Define Variables
			uint32_t Measurement_Raw = 0;
			
			// Combine Read Bytes
			Measurement_Raw = MPL3115A2_Data[0];
			Measurement_Raw <<= 8;
			Measurement_Raw |= MPL3115A2_Data[1];
			Measurement_Raw <<= 8;
			Measurement_Raw |= MPL3115A2_Data[2];
			Measurement_Raw >>= 4;
			
			// Calculate Pressure (mBar)
			Measurement_Array[Read_ID] = (Measurement_Raw / 4.00 ) / 100;

			// Calibration Curve Calculation
			Measurement_Array[Read_ID] = (MPL3115A2_P_Calibrarion_a * Measurement_Array[Read_ID]) + MPL3115A2_P_Calibrarion_b;

			// Read Delay
			delay(512);
			
		}
		
	}

	// ************************************************************
	// Calculate Value for Average Type
	// ************************************************************
	if (AVG_Type == 1) {
		
		// ************************************************************
		// Calculate Aritmetic Average With Measurement Array
		// ************************************************************
		
		// Calculate Average
		double Average_Value_Sum = 0, Average = 0;
		for (int i = 0; i < Read_Count; i++) Average_Value_Sum += Measurement_Array[i];
		Average = Average_Value_Sum / Read_Count;
		
		// Calculate Average
		Value_ = Average;
		
		// Set Deviation Variable
		Deviation_ = 0;
		
	}	// Aritmetic Average
	if (AVG_Type == 2) {
		
		// ************************************************************
		// Calculate RMS Average With Measurement Array
		// ************************************************************
		
		// Calculate Average
		double Average_Value_Sum = 0, Average = 0;
		for (int i = 0; i < Read_Count; i++) Average_Value_Sum += sq(Measurement_Array[i]);
		Average = sqrt(Average_Value_Sum / Read_Count);
		
		// Calculate Average
		Value_ = Average;
		
		// Set Deviation Variable
		Deviation_ = 0;
		
	}	// RMS Average
	if (AVG_Type == 3) {
		
		// ************************************************************
		// Calculate Max & Min values and Extract From Measurement
		// Calculate RMS Average With Filtered Measurement Array
		// ************************************************************
		
		
	}	// Filtered RMS Average
	if (AVG_Type == 4) {
		
		// ************************************************************
		// Calculate Median Point of Measurement Array
		// ************************************************************
		
		
	}	// Median Average
	if (AVG_Type == 5) {
		
		// Calculate Average and Standart Deviation With Measurement Array
		// Select x Sigma Valid Data and Calculate Aritmetic Average
		
		// Define Variables
		int Valid_Data_Count = 0;
		double Value_Sum = 0;
		
		// Calculate Average
		double Average_Value_Sum = 0, Average = 0;
		for (int i = 0; i < Read_Count; i++) Average_Value_Sum += Measurement_Array[i];
		Average = Average_Value_Sum / Read_Count;
		
		// Calculate Standart Deviation
		double Deviation_Value_Sum = 0, Deviation = 0;
		for (int i = 0; i < Read_Count; i++) Deviation_Value_Sum += ((Measurement_Array[i] - Average) * (Measurement_Array[i] - Average));
		Deviation = sqrt(Deviation_Value_Sum / Read_Count);
		
		// Set Sigma Min/Max Values
		float Sigma_1_Max = Average + (1 * Deviation);
		float Sigma_1_Min = Average - (1 * Deviation);
		
		// Handle Array Values
		for (int Calculation_ID = 0; Calculation_ID < Read_Count; Calculation_ID++) {
			
			// Control for 1 Sigma Data
			if (Measurement_Array[Calculation_ID] >= Sigma_1_Min and Measurement_Array[Calculation_ID] <= Sigma_1_Max) {
				
				// Calculate Sum
				Value_Sum += Measurement_Array[Calculation_ID];
				
				// Calculate Valid Data Count
				Valid_Data_Count++;
				
			}
			
		}
		
		// Control for Valid Data
		if (Valid_Data_Count < 1) {
			
			// Set Error Code
			Value_ = -107;
			
			// End Function
			return(false);
			
		}
		
		// Calculate Average
		Value_ = (Value_Sum / Valid_Data_Count);
		
		// Set Deviation Variable
		Deviation_ = Deviation;
		
	}	// 1 Sigma Average

	// ************************************************************
	// Control For Sensor Range
	// ************************************************************
	if (Value_ <= MPL3115A2[0].Range_Min or Value_ >= MPL3115A2[0].Range_Max) {
		
		// Set Error Code
		Value_ = -108;
		
		// End Function
		return(false);
		
	}

	// End Function
	return(true);

}
bool Environment::TSL2561_L(uint8_t Read_Count, uint8_t AVG_Type, float &Value_, float &Deviation_) {
	
	/******************************************************************************
	 *	Project		: TSL2561 Light Read Function
	 *	Developer	: Mehmet Gunce Akkoyun (akkoyun@me.com)
	 *	Revision	: 03.03.00
	 *	Relase		: 24.11.2018
	 *	AVG Type	: 1-AVG, 2-RMS, 3-EXRMS, 4-MEDIAN, 5-Sigma1RMS
	 ******************************************************************************/
	
	// Define Sensor Settings
	int TSL2561_Integrate_Time 	= 1; // 13.7 ms - 0.034 Scale
	int TSL2561_Gain 			= 1; // 1x Gain
	
	// Control Read Count
	if (Read_Count < 1) Read_Count = 1;
	if (Read_Count > 50) Read_Count = 50;
	
	// Control for EXRMS Read Count
	if (Read_Count < 3 and AVG_Type == 3) AVG_Type = 2;
	
	/****************************************
	 * Read Device ID Register from TSL2561
	 ****************************************/
	
	// Request Device ID Register
	Wire.beginTransmission(0b00111001);
	Wire.write(0b10001010); // 0x80 | 0x0A
	
	// Close I2C Connection
	Wire.endTransmission();
	
	// Read Device ID Register
	Wire.requestFrom(0b00111001, 1);
	uint8_t TSL2561_Device_ID = Wire.read();
	
	// 0b0000xxxx = TSL2560
	// 0b0001xxxx = TSL2561
	
	// Control for Device ID
	if (TSL2561_Device_ID == 0b01010000 or TSL2561_Device_ID == 0b11111111) {
		
		/****************************************
		 * Read Timing Register from TSL2561
		 ****************************************/
		
		// Request Timing Register
		Wire.beginTransmission(0b00111001);
		Wire.write(0b10000001); // 0x80 | 0x01
		
		// Close I2C Connection
		Wire.endTransmission(false);
		
		// Read Timing Register
		Wire.requestFrom(0b00111001, 1);
		uint8_t TSL2561_Timing_Register  = Wire.read();
		
		/****************************************
		 * Set Timing & Gain bits
		 ****************************************/
		
		// Set Integrate Time Bit Values
		if (TSL2561_Integrate_Time == 1) {
			
			// Set Bit 0 LOW
			TSL2561_Timing_Register &= 0b11111110;
			
			// Set Bit 1 LOW
			TSL2561_Timing_Register &= 0b11111101;
			
		} // 0.034 Scale - 13.7 ms
		if (TSL2561_Integrate_Time == 2) {
			
			// Set Bit 0 HIGH
			TSL2561_Timing_Register |= 0b00000001;
			
			// Set Bit 1 LOW
			TSL2561_Timing_Register &= 0b11111101;
			
		} // 0.252 Scale - 101 ms
		if (TSL2561_Integrate_Time == 3) {
			
			// Set Bit 0 LOW
			TSL2561_Timing_Register &= 0b11111110;
			
			// Set Bit 1 HIGH
			TSL2561_Timing_Register |= 0b00000010;
			
		} // 1.000 Scale - 402 ms
		
		// Set Gain Bit Values
		if (TSL2561_Gain == 0) {
			
			// Set Bit 4 LOW
			TSL2561_Timing_Register &= 0b11101111;
			
		} // 0 - Low Gain (1x)
		if (TSL2561_Gain == 1) {
			
			// Set Bit 4 HIGH
			TSL2561_Timing_Register |= 0b00010000;
			
		} // 1 - High Gain (16x)
		
		/****************************************
		 * Write Timing Register to TSL2561
		 ****************************************/
		
		// Write Timing Register
		Wire.beginTransmission(0b00111001);
		Wire.write(0b10000001); // 0x80 | 0x01
		Wire.write(TSL2561_Timing_Register);
		
		// Close I2C Connection
		int TSL2561_Timing_Register_Write = Wire.endTransmission(false);
		
		// Control For Register Write
		if (TSL2561_Timing_Register_Write != 0) {
			
			// Set Error Code
			Value_ = -101;
			
			// End Function
			return(false);
			
		}
		
		// Delay
		delay(50);
		
		/****************************************
		 * Power ON TSL2561
		 ****************************************/
		
		// Set Power On Register
		Wire.beginTransmission(0b00111001);
		Wire.write(0b10000000); // 0x80 | 0x00
		Wire.write(0b00000011); // 0x03
		
		// Close I2C Connection
		int TSL2561_Power_ON_Register_Write = Wire.endTransmission(false);
		
		// Control For Register Write
		if (TSL2561_Power_ON_Register_Write != 0) {
			
			// Set Error Code
			Value_ = -102;
			
			// End Function
			return(false);
			
		}
		
		// Define Light Read Array
		unsigned long Lux_Array[Read_Count];
		
		// Read Loop For Read Count
		for (int Read_ID = 0; Read_ID < Read_Count; Read_ID++) {
			
			/****************************************
			 * Read CH0
			 ****************************************/
			
			// Request DATA0LOW Register
			Wire.beginTransmission(0b00111001);
			Wire.write(0b10001100);
			
			// Close I2C Connection
			Wire.endTransmission(false);
			
			// Read DATA0LOW Register
			Wire.requestFrom(0b00111001, 1);
			uint8_t TSL2561_CH0_LSB = Wire.read();
			
			// Request DATA0HIGH Register
			Wire.beginTransmission(0b00111001);
			Wire.write(0b10001101);
			
			// Close I2C Connection
			Wire.endTransmission(false);
			
			// Read DATA0HIGH Register
			Wire.requestFrom(0b00111001, 1);
			uint8_t TSL2561_CH0_MSB = Wire.read();
			
			// Combine Read Bytes
			uint16_t TSL2561_CH0 = (TSL2561_CH0_MSB << 8) | TSL2561_CH0_LSB;
			
			/****************************************
			 * Read CH1
			 ****************************************/
			
			// Request DATA1LOW Register
			Wire.beginTransmission(0b00111001);
			Wire.write(0b10001110);
			
			// Close I2C Connection
			Wire.endTransmission(false);
			
			// Read DATA1LOW Register
			Wire.requestFrom(0b00111001, 1);
			uint8_t TSL2561_CH1_LSB = Wire.read();
			
			// Request DATA1HIGH Register
			Wire.beginTransmission(0b00111001);
			Wire.write(0b10001111);
			
			// Close I2C Connection
			Wire.endTransmission(false);
			
			// Read DATA1HIGH Register
			Wire.requestFrom(0b00111001, 1);
			uint8_t TSL2561_CH1_MSB = Wire.read();
			
			// Combine Read Bytes
			uint16_t TSL2561_CH1 = (TSL2561_CH1_MSB << 8) | TSL2561_CH1_LSB;
			
			/****************************************
			 * Normalize Data
			 ****************************************/
			
			unsigned long TSL2561_Channel_Scale; 	// chScale
			unsigned long TSL2561_Channel_1;		// channel1
			unsigned long TSL2561_Channel_0;		// channel0
			
			// Scale for Integration Time
			switch (TSL2561_Integrate_Time) {
					
				case 1:
					
					TSL2561_Channel_Scale = 0x7517;
					break;
					
				case 2:
					
					TSL2561_Channel_Scale = 0x0FE7;
					break;
					
				case 3:
					
					TSL2561_Channel_Scale = (1 << 10);
					break;
					
				default:
					break;
			}
			
			// Scale for Gain
			if (TSL2561_Gain == 1) TSL2561_Channel_Scale = TSL2561_Channel_Scale;
			if (TSL2561_Gain == 2) TSL2561_Channel_Scale = TSL2561_Channel_Scale << 4;
			
			// Scale Channel Values
			TSL2561_Channel_0 = (TSL2561_CH0 * TSL2561_Channel_Scale) >> 10;
			TSL2561_Channel_1 = (TSL2561_CH1 * TSL2561_Channel_Scale) >> 10;
			
			/****************************************
			 * Calculate LUX
			 ****************************************/
			
			// Find the Ratio of the Channel Values (Channel1/Channel0)
			unsigned long TSL2561_Channel_Ratio = 0;
			if (TSL2561_Channel_0 != 0) TSL2561_Channel_Ratio = (TSL2561_Channel_1 << 10) / TSL2561_Channel_0;
			
			// Round the Ratio Value
			unsigned long TSL2561_Ratio = (TSL2561_Channel_Ratio + 1) >> 1;
			
			unsigned int TSL2561_Calculation_B, TSL2561_Calculation_M;
			
			if ((TSL2561_Ratio >= 0) && (TSL2561_Ratio <= 0x0040)) {
				
				TSL2561_Calculation_B = 0x01F2;
				TSL2561_Calculation_M = 0x01BE;
				
			}
			else if (TSL2561_Ratio <= 0x0080) {
				
				TSL2561_Calculation_B = 0x0214;
				TSL2561_Calculation_M = 0x02D1;
				
			}
			else if (TSL2561_Ratio <= 0x00C0) {
				
				TSL2561_Calculation_B = 0x023F;
				TSL2561_Calculation_M = 0x037B;
				
			}
			else if (TSL2561_Ratio <= 0x0100) {
				
				TSL2561_Calculation_B = 0x0270;
				TSL2561_Calculation_M = 0x03FE;
				
			}
			else if (TSL2561_Ratio <= 0x0138) {
				
				TSL2561_Calculation_B = 0x016F;
				TSL2561_Calculation_M = 0x01FC;
				
			}
			else if (TSL2561_Ratio <= 0x019A) {
				
				TSL2561_Calculation_B = 0x00D2;
				TSL2561_Calculation_M = 0x00FB;
				
			}
			else if (TSL2561_Ratio <= 0x029A) {
				
				TSL2561_Calculation_B = 0x0018;
				TSL2561_Calculation_M = 0x0012;
				
			}
			else if (TSL2561_Ratio > 0x029A) {
				
				TSL2561_Calculation_B = 0x0000;
				TSL2561_Calculation_M = 0x0000;
				
			}
			
			unsigned long TSL2561_Lux_Temp;
			
			// Calculate Temp Lux Value
			TSL2561_Lux_Temp = ((TSL2561_Channel_0 * TSL2561_Calculation_B) - (TSL2561_Channel_1 * TSL2561_Calculation_M));
			
			// Do not Allow Negative Lux Value
			if (TSL2561_Lux_Temp < 0) TSL2561_Lux_Temp = 0;
			
			// Round LSB (2^(LUX_SCALE-1))
			TSL2561_Lux_Temp += (1 << 13);
			
			// Strip Fff Fractional Portion
			Lux_Array[Read_ID] = TSL2561_Lux_Temp >> 14;
			
			/****************************************
			 * Read Delay
			 ****************************************/
			
			// Delay
			if (TSL2561_Integrate_Time == 1) delay(14);
			if (TSL2561_Integrate_Time == 2) delay(102);
			if (TSL2561_Integrate_Time == 3) delay(403);
			
		}
		
		/****************************************
		 * Power OFF TSL2561
		 ****************************************/
		
		// Set Power Off Register
		Wire.beginTransmission(0b00111001);
		Wire.write(0b10000000); // 0x80 | 0x00
		Wire.write(0b00000000); // 0x00
		
		// Close I2C Connection
		int TSL2561_Power_OFF_Register_Write = Wire.endTransmission(false);
		
		// Control For Register Write
		if (TSL2561_Power_OFF_Register_Write != 0) {
			
			// Set Error Code
			Value_ = -103;
			
			// End Function
			return(false);
			
		}
		
		// Power Off Delay
		delay(50);
		
		// ************************************************************
		// Calculate Value for Average Type
		// ************************************************************
		if (AVG_Type == 1) {
			
			// Calculate Aritmetic Average With Measurement Array
			
			// Calculate Average
			double Average_Value_Sum = 0, Average = 0;
			for (int i = 0; i < Read_Count; i++) Average_Value_Sum += Lux_Array[i];
			Average = Average_Value_Sum / Read_Count;
			
			// Calculate Average
			Value_ = Average;
			
			// Set Deviation Variable
			Deviation_ = 0;
			
		}	// Aritmetic Average
		if (AVG_Type == 5) {
			
			// Calculate Average and Standart Deviation With Measurement Array
			// Select x Sigma Valid Data and Calculate Aritmetic Average
			
			// Define Variables
			int Valid_Data_Count = 0;
			double Value_Sum = 0;
			int Start = 5;
			
			// Calculate Average
			double Average_Value_Sum = 0, Average = 0;
			for (int i = Start; i < Read_Count; i++) Average_Value_Sum += Lux_Array[i];
			Average = Average_Value_Sum / (Read_Count - Start);
			
			// Calculate Standart Deviation
			double Deviation_Value_Sum = 0, Deviation = 0;
			for (int i = Start; i < Read_Count; i++) Deviation_Value_Sum += ((Lux_Array[i] - Average) * (Lux_Array[i] - Average));
			Deviation = sqrt(Deviation_Value_Sum / (Read_Count - Start));
			
			// Set Sigma Min/Max Values
			float Sigma_1_Max = Average + (1 * Deviation);
			float Sigma_1_Min = Average - (1 * Deviation);
			
			// Handle Array Values
			for (int Calculation_ID = Start; Calculation_ID < Read_Count; Calculation_ID++) {
				
				// Control for 1 Sigma Data
				if (Lux_Array[Calculation_ID] >= Sigma_1_Min and Lux_Array[Calculation_ID] <= Sigma_1_Max) {
					
					// Calculate Sum
					Value_Sum += Lux_Array[Calculation_ID];
					
					// Calculate Valid Data Count
					Valid_Data_Count++;
					
				}
				
			}
			
			// Control for Valid Data
			if (Valid_Data_Count < 1) {
				
				// Set Error Code
				Value_ = -107;
				
				// End Function
				return(false);
				
			}
			
			// Calculate Average
			Value_ = (Value_Sum / Valid_Data_Count);
			
			// Set Deviation Variable
			Deviation_ = Deviation;
			
		}	// 1 Sigma Average
		
	}
	else {
		
		// Set Error Code
		Value_ = -105;
		
		// End Function
		return(false);
		
	}
	
	// End Function
	return(true);
	
}
bool Environment::HDC2010_T(uint8_t Read_Count, uint8_t AVG_Type, float &Value_, float &Deviation_) {

	/******************************************************************************
	 *	Project		: HDC2010 Sensor Read Function
	 *	Developer	: Mehmet Gunce Akkoyun (akkoyun@me.com)
	 *	Revision	: 01.01.00
	 *	Relase		: 29.09.2020
	 *	AVG Type	: 1-AVG, 2-RMS, 3-EXRMS, 4-MEDIAN, 5-Sigma1RMS
	 *	Sensor Type : 1-Temp, 2-Hum, 3-Temp/Hum
	 ******************************************************************************/

	// Set Sensor Definations
	struct Sensor {
		float		Range_Min;
		float		Range_Max;
		uint8_t		Measurement_Rate;
		uint8_t		Resolution_Temperature;
		uint8_t		Resolution_Humidity;
		
	};
	Sensor HDC2010[] {
		
		-40,		// [Range_Min]
		125,		// [Range_Max]
		5,			// [Measurement_Rate]
		14,			// [Resolution_Temperature]
		14,			// [Resolution_Humidity]

	};

	// Define Function Variables
	bool Sensor_Reset_ = true;
	
	// Control Read Count
	if (Read_Count < 1) Read_Count = 1;
	if (Read_Count > 50) Read_Count = 50;

	// Control Average Type
	if (AVG_Type < 1) AVG_Type = 1;
	if (AVG_Type > 5) AVG_Type = 5;

	// Control for EXRMS Read Count
	if (Read_Count < 5 and AVG_Type == 3) AVG_Type = 2;

	// ************************************************************
	// Reset Sensor
	// ************************************************************

	// Reset Sensor
	if (Sensor_Reset_ == true) {
		
		// ************************************************************
		// Read Current Sensor Settings
		// ************************************************************
		
		// Read User Register of HDC2010
		Wire.beginTransmission(0x40);
		Wire.write(0x0E);
		Wire.requestFrom(0x40, 0x01);
		uint8_t HDC2010_Reset_Read = Wire.read();

		// ************************************************************
		// Reset Sensor
		// ************************************************************
		
		// Set Reset Bit
		HDC2010_Reset_Read = (HDC2010_Reset_Read | 0b10000000);
		
		// Send Soft Reset Command to HDC2010
		Wire.beginTransmission(0x40);
		
		// Set Address
		Wire.write(0x0E);

		// Send Data
		Wire.write(HDC2010_Reset_Read);
		
		// Close I2C Connection
		uint8_t HDC2010_Reset = Wire.endTransmission(false);
		
		// Control For Reset Success
		if (HDC2010_Reset != 0) {
			
			// Set Error Code
			Value_ = -101;
			
			// End Function
			return (false);
			
		}
		
		// Software Reset Delay
		delay(10);
		
	}
	
	// ************************************************************
	// Read Current Sensor Settings
	// ************************************************************
	
	// Connect to Sensor
	Wire.beginTransmission(0x40);
	
	// Set Register Read (Config)
	Wire.write(0x0E);

	// Request Data
	Wire.requestFrom(0x40, 0x01);

	// Read Register
	uint8_t HDC2010_Config_Read = Wire.read();

	// delay
	delay(5);

	// Connect to Sensor
	Wire.beginTransmission(0x40);

	// Set Register Read (Config_Read)
	Wire.write(0x0F);

	// Request Data
	Wire.requestFrom(0x40, 0x01);

	// Read Register
	uint8_t HDC2010_MeasurementConfig_Read = Wire.read();

	// ************************************************************
	// Set Sensor Configurations
	// ************************************************************

	// Set Measurement Mode
	HDC2010_MeasurementConfig_Read = (HDC2010_MeasurementConfig_Read & 0xFC);
	HDC2010_MeasurementConfig_Read = (HDC2010_MeasurementConfig_Read | 0x02);

	// Set Measurement Rate
	switch(HDC2010[0].Measurement_Rate) {

		case 0:	// Manual
			HDC2010_Config_Read = (HDC2010_Config_Read & 0x8F);
			break;
			
		case 1:	// 2 Minutes
			HDC2010_Config_Read = (HDC2010_Config_Read & 0x9F);
			HDC2010_Config_Read = (HDC2010_Config_Read | 0x10);
			break;
			
		case 2:	// 1 Minutes
			HDC2010_Config_Read = (HDC2010_Config_Read & 0xAF);
			HDC2010_Config_Read = (HDC2010_Config_Read | 0x20);
			break;
		
		case 3:	// 10 Second
			HDC2010_Config_Read = (HDC2010_Config_Read & 0xBF);
			HDC2010_Config_Read = (HDC2010_Config_Read | 0x30);
			break;
		
		case 4:	// 5 Second
			HDC2010_Config_Read = (HDC2010_Config_Read & 0xCF);
			HDC2010_Config_Read = (HDC2010_Config_Read | 0x40);
			break;
		
		case 5:	// 1 Second
			HDC2010_Config_Read = (HDC2010_Config_Read & 0xDF);
			HDC2010_Config_Read = (HDC2010_Config_Read | 0x50);
			break;
		
		case 6:	// 0.5 Second
			HDC2010_Config_Read = (HDC2010_Config_Read & 0xEF);
			HDC2010_Config_Read = (HDC2010_Config_Read | 0x60);
			break;
		
		case 7:	// 0.2 Second
			HDC2010_Config_Read = (HDC2010_Config_Read | 0x70);
			break;
			
		default:
			HDC2010_Config_Read = (HDC2010_Config_Read & 0x8F);
		
	}
	
	// Set Temperature Resolution
	switch(HDC2010[0].Resolution_Temperature) {
		
		case 14:	// 14 Bit
			HDC2010_MeasurementConfig_Read = (HDC2010_MeasurementConfig_Read & 0x3F);
			break;
			
		case 11:	// 11 Bit
			HDC2010_MeasurementConfig_Read = (HDC2010_MeasurementConfig_Read & 0x7F);
			HDC2010_MeasurementConfig_Read = (HDC2010_MeasurementConfig_Read | 0x40);
			break;
			
		case 9:		// 9 Bit
			HDC2010_MeasurementConfig_Read = (HDC2010_MeasurementConfig_Read & 0xBF);
			HDC2010_MeasurementConfig_Read = (HDC2010_MeasurementConfig_Read | 0x80);
			break;
			
		default:
			HDC2010_MeasurementConfig_Read = (HDC2010_MeasurementConfig_Read & 0x3F);
			
	}

	// Set Humidity Resolution
	switch(HDC2010[0].Resolution_Humidity) {
		
		case 14:	// 14 Bit
			HDC2010_MeasurementConfig_Read = (HDC2010_MeasurementConfig_Read & 0xCF);
			break;
			
		case 11:	// 11 Bit
			HDC2010_MeasurementConfig_Read = (HDC2010_MeasurementConfig_Read & 0xDF);
			HDC2010_MeasurementConfig_Read = (HDC2010_MeasurementConfig_Read | 0x10);
			break;
			
		case 9:		// 9 Bit
			HDC2010_MeasurementConfig_Read = (HDC2010_MeasurementConfig_Read & 0xEF);
			HDC2010_MeasurementConfig_Read = (HDC2010_MeasurementConfig_Read | 0x20);
			break;
			
		default:
			HDC2010_MeasurementConfig_Read = (HDC2010_MeasurementConfig_Read & 0xCF);
			
	}

	// Trigger Measurement
	HDC2010_MeasurementConfig_Read = (HDC2010_MeasurementConfig_Read | 0x01);

	// ************************************************************
	// Write Sensor Configurations
	// ************************************************************

	// Connect to Sensor
	Wire.beginTransmission(0x40);
	
	// Set Register
	Wire.write(0x0E);

	// Send Data
	Wire.write(HDC2010_Config_Read);

	// Close I2C Connection
	uint8_t HDC2010_Config = Wire.endTransmission(false);
	
	// Control For Write Success
	if (HDC2010_Config != 0) {
		
		// Set Error Code
		Value_ = -102;
		
		// End Function
		return (false);
		
	}

	// delay
	delay(5);

	// Send Mode Command to HDC2010
	Wire.beginTransmission(0x40);
	
	// Set Address
	Wire.write(0x0F);

	// Send Data
	Wire.write(HDC2010_MeasurementConfig_Read);

	// Close I2C Connection
	uint8_t HDC2010_Measurement_Config = Wire.endTransmission(false);
	
	// Control For Write Success
	if (HDC2010_Measurement_Config != 0) {
		
		// Set Error Code
		Value_ = -103;
		
		// End Function
		return (false);
		
	}

	// ************************************************************
	// Read Temperature Array
	// ************************************************************

	// Define Measurement Read Array
	float Measurement_Array[Read_Count];

	// Read Loop For Read Count
	for (uint8_t Read_ID = 0; Read_ID < Read_Count; Read_ID++) {
			
		// Define Variables
		uint16_t Measurement_Raw;
		uint8_t HDC2010_Data[2];
			
		// ************************************************************
		// Read Temperature LSB Data
		// ************************************************************

		// Connect to Sensor
		Wire.beginTransmission(0x40);

		// Set Register // LSB Temperature
		Wire.write(0x00);
	
		// Close I2C Connection
		uint8_t HDC2010_Measurement_Low_Read = Wire.endTransmission(false);

		// Control For Read Success
		if (HDC2010_Measurement_Low_Read != 0) {
				
			// Set Error Code
			Value_ = -104;
				
			// End Function
			return (false);
				
		}
			
		// Read LSB Data from HDC2010
		Wire.requestFrom(0x40, 0x01);
			
		// Read Data
		HDC2010_Data[0] = Wire.read(); // LSB Data
			
		// ************************************************************
		// Read Temperature MSB Data
		// ************************************************************

		// Connect to Sensor
		Wire.beginTransmission(0x40);

		// Set Register // MSB Temperature
		Wire.write(0x01);

		// Close I2C Connection
		uint8_t HDC2010_Measurement_High_Read = Wire.endTransmission(false);

		// Control For Read Success
		if (HDC2010_Measurement_High_Read != 0) {
						
			// Set Error Code
			Value_ = -105;
						
			// End Function
			return (false);
				
		}
					
		// Read MSB Data from HDC2010
		Wire.requestFrom(0x40, 0x01);
					
		// Read Data
		HDC2010_Data[1] = Wire.read(); // MSB Data

		// ************************************************************
		// Combine Data
		// ************************************************************

		// Combine Read Bytes
		Measurement_Raw = ((uint16_t)(HDC2010_Data[1]) << 8 | (uint16_t)HDC2010_Data[0]);

		// Calculate Measurement
		Measurement_Array[Read_ID] = (float)Measurement_Raw * 165 / 65536 - 40;

		// Calibrate Measurement
		Measurement_Array[Read_ID] = (HDC2010_T_Calibrarion_a * Measurement_Array[Read_ID]) + HDC2010_T_Calibrarion_b;

	}

	// ************************************************************
	// Calculate Stats
	// ************************************************************

	// Calculate Array Stats
	float Max = double(Measurement_Array[0]); for (int i=0; i < Read_Count; i++) {if (double(Measurement_Array[i]) > Max) Max = double(Measurement_Array[i]);}
	float Min = double(Measurement_Array[0]); for (int i=0; i < Read_Count; i++) {if (double(Measurement_Array[i]) < Min) Min = double(Measurement_Array[i]);}
	float Avg; for (int i=0; i < Read_Count; i++) {Avg += double(Measurement_Array[i]);} Avg /= Read_Count;
	float SDev; for (int i=0; i < Read_Count; i++) {SDev += (double(Measurement_Array[i]) - Avg) * (double(Measurement_Array[i]) - Avg);} SDev = sqrt(SDev/Read_Count);

	// Define Variables
	double Measurement_Sum = 0;
	uint8_t Valid_Data_Count = 0;

	// Calculate Average Data
	for (int Calculation_ID = 0; Calculation_ID < Read_Count; Calculation_ID++) {
		
		// Control Data for Limits
		if (double(Measurement_Array[Calculation_ID]) >= HDC2010[0].Range_Min and double(Measurement_Array[Calculation_ID]) <= HDC2010[0].Range_Max) {
			
			// Calculate RMS/EXRMS Average
			if (AVG_Type == 2 or AVG_Type == 3) {
				
				// Calculate Sum
				Measurement_Sum += sq(double(Measurement_Array[Calculation_ID]));
				
				// Calculate Valid Data Count
				Valid_Data_Count++;
				
			}
			
			// Calulate Sigma1 Average
			if (AVG_Type == 5) {
				
				float Sigma_1_Max = Avg + SDev;
				float Sigma_1_Min = Avg - SDev;
				
				if (Measurement_Array[Calculation_ID] >= Sigma_1_Min and Measurement_Array[Calculation_ID] <= Sigma_1_Max) {
					
					// Calculate Sum
					Measurement_Sum += sq(Measurement_Array[Calculation_ID]);
					
					// Calculate Valid Data Count
					Valid_Data_Count++;
					
				}
				
			}
			
		}
		
	}
	
	// Control for Valid Data
	if (AVG_Type != 1 and Valid_Data_Count < 1) {
		
		// Set Error Code
		Value_ = -106;
		
		// End Function
		return(false);
		
	}

	// ************************************************************
	// Calculate Value
	// ************************************************************

	// Calculate Average
	if (AVG_Type == 1) {
		
		// Calculate Average
		Value_ = Avg;			// Average
		
		// Set Deviation Variable
		Deviation_ = 0;
		
	}	// Standart Average
	if (AVG_Type == 2) {
		
		// Calculate Average
		Value_ = sqrt(Measurement_Sum / Valid_Data_Count);		// RMS
		
		// Set Deviation Variable
		Deviation_ = 0;

	}	// RMS Average
	if (AVG_Type == 3) {
		
		// Eleminate Max Value
		Measurement_Sum = Measurement_Sum - (Max * Max);
		
		// Eleminate Min Value
		Measurement_Sum = Measurement_Sum - (Min * Min);
		
		// Eleminate Min/Max Valid Data Count
		Valid_Data_Count = Valid_Data_Count - 2;
		
		// Calculate Average
		Value_ = sqrt(Measurement_Sum / Valid_Data_Count);		// EXRMS
		
		// Set Deviation Variable
		Deviation_ = 0;

	}	// Extendet RMS Average
	if (AVG_Type == 4) {
		
		// Calculate Average
		Value_ = (Max + Min) / 2;		// Median
		
		// Set Deviation Variable
		Deviation_ = 0;

	}	// Median Average
	if (AVG_Type == 5) {
		
		// Calculate Average
		Value_ = sqrt(Measurement_Sum / Valid_Data_Count);		// RMS
		
		// Set Deviation Variable
		Deviation_ = SDev;

	}	// Sigma1RMS Average
	
	// Handle Measurement
	if (Value_ < HDC2010[0].Range_Min) Value_ = HDC2010[0].Range_Min;
	if (Value_ > HDC2010[0].Range_Max) Value_ = HDC2010[0].Range_Max;
	
	// End Function
	return(true);

}
bool Environment::HDC2010_H(uint8_t Read_Count, uint8_t AVG_Type, float &Value_, float &Deviation_) {

	/******************************************************************************
	 *	Project		: HDC2010 Sensor Read Function
	 *	Developer	: Mehmet Gunce Akkoyun (akkoyun@me.coj)
	 *	Revision	: 01.00.00
	 *	Relase		: 29.09.2020
	 *	AVG Type	: 1-AVG, 2-RMS, 3-EXRMS, 4-MEDIAN, 5-Sigma1RMS
	 *	Sensor Type : 1-Temp, 2-Hum, 3-Temp/Hum
	 ******************************************************************************/

	// Set Sensor Definations
	struct Sensor {
		float		Range_Min;
		float		Range_Max;
		uint8_t		Measurement_Rate;
		uint8_t		Resolution_Temperature;
		uint8_t		Resolution_Humidity;
		
	};
	Sensor HDC2010[] {
		
		0,			// [Range_Min]
		100,		// [Range_Max]
		5,			// [Measurement_Rate]
		14,			// [Resolution_Temperature]
		14,			// [Resolution_Humidity]

	};
	
	// Define Function Variables
	bool Sensor_Reset_ = true;
	
	// Control Read Count
	if (Read_Count < 1) Read_Count = 1;
	if (Read_Count > 50) Read_Count = 50;

	// Control Average Type
	if (AVG_Type < 1) AVG_Type = 1;
	if (AVG_Type > 5) AVG_Type = 5;

	// Control for EXRMS Read Count
	if (Read_Count < 5 and AVG_Type == 3) AVG_Type = 2;

	// ************************************************************
	// Reset Sensor
	// ************************************************************

	// Reset Sensor
	if (Sensor_Reset_ == true) {
		
		// ************************************************************
		// Read Current Sensor Settings
		// ************************************************************
		
		// Read User Register of HDC2010
		Wire.beginTransmission(0x40);
		Wire.write(0x0E);
		Wire.requestFrom(0x40, 0x01);
		uint8_t HDC2010_Reset_Read = Wire.read();

		// ************************************************************
		// Reset Sensor
		// ************************************************************
		
		// Set Reset Bit
		HDC2010_Reset_Read = (HDC2010_Reset_Read | 0b10000000);
		
		// Send Soft Reset Command to HDC2010
		Wire.beginTransmission(0x40);
		
		// Set Address
		Wire.write(0x0E);

		// Send Data
		Wire.write(HDC2010_Reset_Read);
		
		// Close I2C Connection
		uint8_t HDC2010_Reset = Wire.endTransmission(false);
		
		// Control For Reset Success
		if (HDC2010_Reset != 0) {
			
			// Set Error Code
			Value_ = -101;
			
			// End Function
			return (false);
			
		}
		
		// Software Reset Delay
		delay(50);
		
	}
	
	// ************************************************************
	// Read Current Sensor Settings
	// ************************************************************
	
	// Connect to Sensor
	Wire.beginTransmission(0x40);
	
	// Set Register Read (Config)
	Wire.write(0x0E);

	// Request Data
	Wire.requestFrom(0x40, 0x01);

	// Read Register
	uint8_t HDC2010_Config_Read = Wire.read();

	// delay
	delay(10);

	// Connect to Sensor
	Wire.beginTransmission(0x40);

	// Set Register Read (Config_Read)
	Wire.write(0x0F);

	// Request Data
	Wire.requestFrom(0x40, 0x01);

	// Read Register
	uint8_t HDC2010_MeasurementConfig_Read = Wire.read();
	
	// ************************************************************
	// Set Sensor Configurations
	// ************************************************************

	// Set Measurement Mode
	HDC2010_MeasurementConfig_Read = (HDC2010_MeasurementConfig_Read & 0xFD);
	HDC2010_MeasurementConfig_Read = (HDC2010_MeasurementConfig_Read | 0x04);

	// Set Measurement Rate
	switch(HDC2010[0].Measurement_Rate) {

		case 0:	// Manual
			HDC2010_Config_Read = (HDC2010_Config_Read & 0x8F);
			break;
			
		case 1:	// 2 Minutes
			HDC2010_Config_Read = (HDC2010_Config_Read & 0x9F);
			HDC2010_Config_Read = (HDC2010_Config_Read | 0x10);
			break;
			
		case 2:	// 1 Minutes
			HDC2010_Config_Read = (HDC2010_Config_Read & 0xAF);
			HDC2010_Config_Read = (HDC2010_Config_Read | 0x20);
			break;
		
		case 3:	// 10 Second
			HDC2010_Config_Read = (HDC2010_Config_Read & 0xBF);
			HDC2010_Config_Read = (HDC2010_Config_Read | 0x30);
			break;
		
		case 4:	// 5 Second
			HDC2010_Config_Read = (HDC2010_Config_Read & 0xCF);
			HDC2010_Config_Read = (HDC2010_Config_Read | 0x40);
			break;
		
		case 5:	// 1 Second
			HDC2010_Config_Read = (HDC2010_Config_Read & 0xDF);
			HDC2010_Config_Read = (HDC2010_Config_Read | 0x50);
			break;
		
		case 6:	// 0.5 Second
			HDC2010_Config_Read = (HDC2010_Config_Read & 0xEF);
			HDC2010_Config_Read = (HDC2010_Config_Read | 0x60);
			break;
		
		case 7:	// 0.2 Second
			HDC2010_Config_Read = (HDC2010_Config_Read | 0x70);
			break;
			
		default:
			HDC2010_Config_Read = (HDC2010_Config_Read & 0x8F);
		
	}
	
	// Set Temperature Resolution
	switch(HDC2010[0].Resolution_Temperature) {
		
		case 14:	// 14 Bit
			HDC2010_MeasurementConfig_Read = (HDC2010_MeasurementConfig_Read & 0x3F);
			break;
			
		case 11:	// 11 Bit
			HDC2010_MeasurementConfig_Read = (HDC2010_MeasurementConfig_Read & 0x7F);
			HDC2010_MeasurementConfig_Read = (HDC2010_MeasurementConfig_Read | 0x40);
			break;
			
		case 9:		// 9 Bit
			HDC2010_MeasurementConfig_Read = (HDC2010_MeasurementConfig_Read & 0xBF);
			HDC2010_MeasurementConfig_Read = (HDC2010_MeasurementConfig_Read | 0x80);
			break;
			
		default:
			HDC2010_MeasurementConfig_Read = (HDC2010_MeasurementConfig_Read & 0x3F);
			
	}

	// Set Humidity Resolution
	switch(HDC2010[0].Resolution_Humidity) {
		
		case 14:	// 14 Bit
			HDC2010_MeasurementConfig_Read = (HDC2010_MeasurementConfig_Read & 0xCF);
			break;
			
		case 11:	// 11 Bit
			HDC2010_MeasurementConfig_Read = (HDC2010_MeasurementConfig_Read & 0xDF);
			HDC2010_MeasurementConfig_Read = (HDC2010_MeasurementConfig_Read | 0x10);
			break;
			
		case 9:		// 9 Bit
			HDC2010_MeasurementConfig_Read = (HDC2010_MeasurementConfig_Read & 0xEF);
			HDC2010_MeasurementConfig_Read = (HDC2010_MeasurementConfig_Read | 0x20);
			break;
			
		default:
			HDC2010_MeasurementConfig_Read = (HDC2010_MeasurementConfig_Read & 0xCF);
			
	}

	// Trigger Measurement
	HDC2010_MeasurementConfig_Read = (HDC2010_MeasurementConfig_Read | 0x01);

	// ************************************************************
	// Write Sensor Configurations
	// ************************************************************

	// Connect to Sensor
	Wire.beginTransmission(0x40);
	
	// Set Register
	Wire.write(0x0E);

	// Send Data
	Wire.write(HDC2010_Config_Read);

	// Close I2C Connection
	uint8_t HDC2010_Config = Wire.endTransmission(false);
	
	// Control For Write Success
	if (HDC2010_Config != 0) {
		
		// Set Error Code
		Value_ = -102;
		
		// End Function
		return (false);
		
	}

	// delay
	delay(5);

	// Send Mode Command to HDC2010
	Wire.beginTransmission(0x40);
	
	// Set Address
	Wire.write(0x0F);

	// Send Data
	Wire.write(HDC2010_MeasurementConfig_Read);

	// Close I2C Connection
	uint8_t HDC2010_Measurement_Config = Wire.endTransmission(false);
	
	// Control For Write Success
	if (HDC2010_Measurement_Config != 0) {
		
		// Set Error Code
		Value_ = -103;
		
		// End Function
		return (false);
		
	}

	// ************************************************************
	// Read Temperature Array
	// ************************************************************

	// Define Measurement Read Array
	float Measurement_Array[Read_Count];

	// Read Loop For Read Count
	for (uint8_t Read_ID = 0; Read_ID < Read_Count; Read_ID++) {
			
		// Define Variables
		uint16_t Measurement_Raw;
		uint8_t HDC2010_Data[2];
			
		// ************************************************************
		// Read Humidity LSB Data
		// ************************************************************

		// Connect to Sensor
		Wire.beginTransmission(0x40);

		// Set Register // LSB Humidity
		Wire.write(0x02);
	
		// Close I2C Connection
		uint8_t HDC2010_Measurement_Low_Read = Wire.endTransmission(false);

		// Control For Read Success
		if (HDC2010_Measurement_Low_Read != 0) {
				
			// Set Error Code
			Value_ = -104;
				
			// End Function
			return (false);
				
		}
			
		// Read LSB Data from HDC2010
		Wire.requestFrom(0x40, 0x01);
			
		// Read Data
		HDC2010_Data[0] = Wire.read(); // LSB Data
			
		// ************************************************************
		// Read Humidity MSB Data
		// ************************************************************

		// Connect to Sensor
		Wire.beginTransmission(0x40);

		// Set Register // MSB Humidity
		Wire.write(0x03);

		// Close I2C Connection
		uint8_t HDC2010_Measurement_High_Read = Wire.endTransmission(false);

		// Control For Read Success
		if (HDC2010_Measurement_High_Read != 0) {
						
			// Set Error Code
			Value_ = -105;
						
			// End Function
			return (false);
				
		}
					
		// Read MSB Data from HDC2010
		Wire.requestFrom(0x40, 0x01);
					
		// Read Data
		HDC2010_Data[1] = Wire.read(); // MSB Data

		// ************************************************************
		// Combine Data
		// ************************************************************

		// Combine Read Bytes
		Measurement_Raw = ((uint16_t)(HDC2010_Data[1]) << 8 | (uint16_t)HDC2010_Data[0]);

		// Calculate Measurement
		Measurement_Array[Read_ID] = (float) Measurement_Raw / (65536) * 100;

		// Calibrate Measurement
		Measurement_Array[Read_ID] = (HDC2010_H_Calibrarion_a * Measurement_Array[Read_ID]) + HDC2010_H_Calibrarion_b;

	}

	// ************************************************************
	// Calculate Stats
	// ************************************************************

	// Calculate Array Stats
	float Max = double(Measurement_Array[0]); for (int i=0; i < Read_Count; i++) {if (double(Measurement_Array[i]) > Max) Max = double(Measurement_Array[i]);}
	float Min = double(Measurement_Array[0]); for (int i=0; i < Read_Count; i++) {if (double(Measurement_Array[i]) < Min) Min = double(Measurement_Array[i]);}
	float Avg; for (int i=0; i < Read_Count; i++) {Avg += double(Measurement_Array[i]);} Avg /= Read_Count;
	float SDev; for (int i=0; i < Read_Count; i++) {SDev += (double(Measurement_Array[i]) - Avg) * (double(Measurement_Array[i]) - Avg);} SDev = sqrt(SDev/Read_Count);

	// Define Variables
	double Measurement_Sum = 0;
	uint8_t Valid_Data_Count = 0;

	// Calculate Average Data
	for (int Calculation_ID = 0; Calculation_ID < Read_Count; Calculation_ID++) {
		
		// Control Data for Limits
		if (double(Measurement_Array[Calculation_ID]) >= HDC2010[0].Range_Min and double(Measurement_Array[Calculation_ID]) <= HDC2010[0].Range_Max) {
			
			// Calculate RMS/EXRMS Average
			if (AVG_Type == 2 or AVG_Type == 3) {
				
				// Calculate Sum
				Measurement_Sum += sq(double(Measurement_Array[Calculation_ID]));
				
				// Calculate Valid Data Count
				Valid_Data_Count++;
				
			}
			
			// Calulate Sigma1 Average
			if (AVG_Type == 5) {
				
				float Sigma_1_Max = Avg + SDev;
				float Sigma_1_Min = Avg - SDev;
				
				if (Measurement_Array[Calculation_ID] >= Sigma_1_Min and Measurement_Array[Calculation_ID] <= Sigma_1_Max) {
					
					// Calculate Sum
					Measurement_Sum += sq(Measurement_Array[Calculation_ID]);
					
					// Calculate Valid Data Count
					Valid_Data_Count++;
					
				}
				
			}
			
		}
		
	}
	
	// Control for Valid Data
	if (AVG_Type != 1 and Valid_Data_Count < 1) {
		
		// Set Error Code
		Value_ = -106;
		
		// End Function
		return(false);
		
	}

	// ************************************************************
	// Calculate Value
	// ************************************************************

	// Calculate Average
	if (AVG_Type == 1) {
		
		// Calculate Average
		Value_ = Avg;			// Average
		
		// Set Deviation Variable
		Deviation_ = 0;
		
	}	// Standart Average
	if (AVG_Type == 2) {
		
		// Calculate Average
		Value_ = sqrt(Measurement_Sum / Valid_Data_Count);		// RMS
		
		// Set Deviation Variable
		Deviation_ = 0;

	}	// RMS Average
	if (AVG_Type == 3) {
		
		// Eleminate Max Value
		Measurement_Sum = Measurement_Sum - (Max * Max);
		
		// Eleminate Min Value
		Measurement_Sum = Measurement_Sum - (Min * Min);
		
		// Eleminate Min/Max Valid Data Count
		Valid_Data_Count = Valid_Data_Count - 2;
		
		// Calculate Average
		Value_ = sqrt(Measurement_Sum / Valid_Data_Count);		// EXRMS
		
		// Set Deviation Variable
		Deviation_ = 0;

	}	// Extendet RMS Average
	if (AVG_Type == 4) {
		
		// Calculate Average
		Value_ = (Max + Min) / 2;		// Median
		
		// Set Deviation Variable
		Deviation_ = 0;

	}	// Median Average
	if (AVG_Type == 5) {
		
		// Calculate Average
		Value_ = sqrt(Measurement_Sum / Valid_Data_Count);		// RMS
		
		// Set Deviation Variable
		Deviation_ = SDev;

	}	// Sigma1RMS Average
	
	// Handle Measurement
	if (Value_ < HDC2010[0].Range_Min) Value_ = HDC2010[0].Range_Min;
	if (Value_ > HDC2010[0].Range_Max) Value_ = HDC2010[0].Range_Max;
	
	// End Function
	return(true);

}




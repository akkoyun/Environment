/* *******************************************************************************
 *  Copyright (C) 2014-2019 Mehmet Gunce Akkoyun Can not be copied and/or
 *	distributed without the express permission of Mehmet Gunce Akkoyun
 *	This library is a combined book of environment sensor library.
 *
 *	Library				: Environment Library.
 *	Code Developer		: Mehmet Gunce Akkoyun (akkoyun@me.com)
 *********************************************************************************/

#include "Environment.h"

// HDC2010 Functions
HDC2010::HDC2010(bool _Multiplexer_Enable, uint8_t _Multiplexer_Channel, uint8_t _Measurement_Count, bool _Calibration_Enable) {

	// Set Measurement Count
	this->_Read_Count = _Measurement_Count;

	// Enable Calibration
	this->_Calibration = _Calibration_Enable;

	// Set Multiplexer Variables
	this->_Mux_Enable = _Multiplexer_Enable;
	this->_Mux_Channel = _Multiplexer_Channel;

}
float HDC2010::Temperature(void) {

	// Create I2C Device
	I2C_Functions HDC2010_Temperature(__I2C_Addr_HDC2010__, this->_Mux_Enable, this->_Mux_Channel);

	// Reset Sensor
	HDC2010_Temperature.Set_Register_Bit(0x0E, 7, false);

	// Read Register
	uint8_t HDC2010_Config_Read = HDC2010_Temperature.Read_Register(0x0E);

	// Read Register
	uint8_t HDC2010_MeasurementConfig_Read = HDC2010_Temperature.Read_Register(0x0F);

	// Set Measurement Rate
	HDC2010_Config_Read &= 0x8F;

	// Set Measurement Mode
	HDC2010_MeasurementConfig_Read &= 0xFC;
	HDC2010_MeasurementConfig_Read |= 0x02;

	// Set Temperature Resolution (9 bit)
	HDC2010_MeasurementConfig_Read &= 0xBF;
	HDC2010_MeasurementConfig_Read |= 0x80;

	// Set Humidity Resolution (9 bit)
	HDC2010_MeasurementConfig_Read &= 0xEF;
	HDC2010_MeasurementConfig_Read |= 0x20;

	// Trigger Measurement
	HDC2010_MeasurementConfig_Read |= 0x01;

	// Write Register
	HDC2010_Temperature.Write_Register(0x0E, HDC2010_Config_Read, false);

	// Write Register
	HDC2010_Temperature.Write_Register(0x0F, HDC2010_MeasurementConfig_Read, false);

	// Define Measurement Read Array
	float Measurement_Array[_Read_Count];

	// Read Loop For Read Count
	for (int Read_ID = 0; Read_ID < _Read_Count; Read_ID++) {

		// Define Variables
		uint8_t HDC2010_Data[2];

		// Read Delay
		delay(5);

		// Read Register
		HDC2010_Data[0] = HDC2010_Temperature.Read_Register(0x00);
		HDC2010_Data[1] = HDC2010_Temperature.Read_Register(0x01);

		// Combine Read Bytes
		uint16_t Measurement_Raw = ((uint16_t)(HDC2010_Data[1]) << 8 | (uint16_t)HDC2010_Data[0]);

		// Calculate Measurement
		Measurement_Array[Read_ID] = (float)Measurement_Raw * 165 / 65536 - 40;

	}

	// Construct Object
	Array_Stats<float> Data_Array(Measurement_Array, _Read_Count);

	// Calculate Average
	float Value_ = Data_Array.Average(Data_Array.Arithmetic_Avg);

	// Calibrate Data
	if (_Calibration) Value_ = (1.0053 * Value_) -0.4102;

	// Control For Sensor Range
	if (Value_ < -40 or Value_ > 125) Value_ = -101;

	// End Function
	return(Value_);

}
float HDC2010::Humidity(void) {

	// Create I2C Device
	I2C_Functions HDC2010_Humidity(__I2C_Addr_HDC2010__, this->_Mux_Enable, this->_Mux_Channel);

	// Reset Sensor
	HDC2010_Humidity.Set_Register_Bit(0x0E, 7, false);

	// Read Register
	uint8_t HDC2010_Config_Read = HDC2010_Humidity.Read_Register(0x0E);

	// Read Register
	uint8_t HDC2010_MeasurementConfig_Read = HDC2010_Humidity.Read_Register(0x0F);

	// Set Measurement Rate
	HDC2010_Config_Read &= 0xDF;
	HDC2010_Config_Read |= 0x50;

	// Set Measurement Mode
	HDC2010_MeasurementConfig_Read &= 0xFD;
	HDC2010_MeasurementConfig_Read |= 0x04;

	// Set Temperature Resolution (9 bit)
	HDC2010_MeasurementConfig_Read &= 0xBF;
	HDC2010_MeasurementConfig_Read |= 0x80;

	// Set Humidity Resolution (9 bit)
	HDC2010_MeasurementConfig_Read &= 0xEF;
	HDC2010_MeasurementConfig_Read |= 0x20;

	// Trigger Measurement
	HDC2010_MeasurementConfig_Read |= 0x01;

	// Write Register
	HDC2010_Humidity.Write_Register(0x0E, HDC2010_Config_Read, false);

	// Write Register
	HDC2010_Humidity.Write_Register(0x0F, HDC2010_MeasurementConfig_Read, false);

	// Define Measurement Read Array
	float Measurement_Array[_Read_Count];

	// Read Loop For Read Count
	for (int Read_ID = 0; Read_ID < _Read_Count; Read_ID++) {

		// Define Variables
		uint8_t HDC2010_Data[2];

		// Read Delay
		delay(5);

		// Read Register
		HDC2010_Data[0] = HDC2010_Humidity.Read_Register(0x02);
		HDC2010_Data[1] = HDC2010_Humidity.Read_Register(0x03);

		// Combine Read Bytes
		uint16_t Measurement_Raw = ((uint16_t)(HDC2010_Data[1]) << 8 | (uint16_t)HDC2010_Data[0]);

		// Calculate Measurement
		Measurement_Array[Read_ID] = (float)Measurement_Raw / 65536 * 100;

	}

	// Construct Object
	Array_Stats<float> Data_Array(Measurement_Array, _Read_Count);

	// Calculate Average
	float Value_ = Data_Array.Average(Data_Array.Arithmetic_Avg);

	// Calibrate Data
	if (_Calibration) Value_ = (0.9821 * Value_) -0.3217;

	// Control For Sensor Range
	if (Value_ < 0 or Value_ > 100) Value_ = -101;

	// End Function
	return(Value_);

}

// SHT21 Functions
SHT21::SHT21(bool _Multiplexer_Enable, uint8_t _Multiplexer_Channel, uint8_t _Measurement_Count, bool _Calibration_Enable) {

	// Set Measurement Count
	this->_Read_Count = _Measurement_Count;

	// Enable Calibration
	this->_Calibration = _Calibration_Enable;

	// Set Multiplexer Variables
	this->_Mux_Enable = _Multiplexer_Enable;
	this->_Mux_Channel = _Multiplexer_Channel;

}
float SHT21::Temperature(void) {

	// Create I2C Device
	I2C_Functions SHT21_Temperature(__I2C_Addr_SHT21__, this->_Mux_Enable, this->_Mux_Channel);

	// User Register Definition
	uint8_t User_Reg_ = 0b01000010;
	
	// Send Soft Reset Command to SHT21
	SHT21_Temperature.Write_Command(0xFE, false);

	// Read User Register of SHT21
	uint8_t SHT21_Config_Read = SHT21_Temperature.Read_Register(0xE6);

	// Control for Config Read Register
	if (SHT21_Config_Read != User_Reg_) if (!SHT21_Temperature.Write_Register(0xE6, User_Reg_, false)) return(-102);

	// Define Measurement Read Array
	float Measurement_Array[this->_Read_Count];

	// Read Loop For Read Count
	for (uint8_t Read_ID = 0; Read_ID < this->_Read_Count; Read_ID++) {

		// Define Data Variable
		uint8_t SHT21_Data[4];

		// Send Read Command to SHT21
		SHT21_Temperature.Read_Multiple_Register(0xE3, SHT21_Data, 3, false);

		// Combine Read Bytes
		uint16_t Measurement_Raw = ((uint16_t)SHT21_Data[0] << 8) | (uint16_t)SHT21_Data[1];
		
		// Clear 2 Low Status Bit
		Measurement_Raw &= ~0x0003;
		
		// Calculate Measurement
		Measurement_Array[Read_ID] = -46.85 + 175.72 * (float)Measurement_Raw / pow(2,16);

		// Clear Buffer Array
		memset(SHT21_Data, '\0', 4);
			
	}

	// Construct Object
	Array_Stats<float> Data_Array(Measurement_Array, this->_Read_Count);

	// Calculate Average
	float Value_ = Data_Array.Average(Data_Array.Arithmetic_Avg);

	// Control Limits
	if (Value_ < -40 or Value_ > 125) Value_ = -101;

	// Calibrate Data
	if (this->_Calibration) Value_ = (1.0129 * Value_) + 0.6075;

	// End Function
	return(Value_);

}
float SHT21::Humidity(void) {

	// Create I2C Device
	I2C_Functions SHT21_Humidity(__I2C_Addr_SHT21__, this->_Mux_Enable, this->_Mux_Channel);

	// User Register Definition
	uint8_t User_Reg_ = 0b00000001;
	
	// Send Soft Reset Command to SHT21
	SHT21_Humidity.Write_Command(0xFE, false);

	// Read User Register of SHT21
	uint8_t SHT21_Config_Read = SHT21_Humidity.Read_Register(0xE6);

	// Control for Config Read Register
	if (SHT21_Config_Read != User_Reg_) if (!SHT21_Humidity.Write_Register(0xE6, User_Reg_, false)) return(-102);

	// Define Measurement Read Array
	float Measurement_Array[this->_Read_Count];

	// Read Loop For Read Count
	for (uint8_t Read_ID = 0; Read_ID < this->_Read_Count; Read_ID++) {

		// Define Data Variable
		uint8_t SHT21_Data[4];

		// Send Read Command to SHT21
		SHT21_Humidity.Read_Multiple_Register(0xE5, SHT21_Data, 3, false);

		// Combine Read Bytes
		uint16_t Measurement_Raw = ((uint16_t)SHT21_Data[0] << 8) | (uint16_t)SHT21_Data[1];
		
		// Clear 2 Low Status Bit
		Measurement_Raw &= ~0x0005;
		
		// Calculate Measurement
		Measurement_Array[Read_ID] = -6 + 125 * (float)Measurement_Raw / pow(2,16);

		// Clear Buffer Array
		memset(SHT21_Data, '\0', 4);
			
	}

	// Construct Object
	Array_Stats<float> Data_Array(Measurement_Array, this->_Read_Count);

	// Calculate Average
	float Value_ = Data_Array.Average(Data_Array.Arithmetic_Avg);

	// Control Limits
	if (Value_ < 0 or Value_ > 100) Value_ = -101;

	// Calibrate Data
	if (this->_Calibration) Value_ = (0.9518 * Value_) + 3.5316;

	// End Function
	return(Value_);

}

// MPL3115A2 Functions
MPL3115A2::MPL3115A2(uint8_t _Measurement_Count, bool _Calibration_Enable) {

	// Set Measurement Count
	this->_Read_Count = _Measurement_Count;

	// Enable Calibration
	this->_Calibration = _Calibration_Enable;

}
float MPL3115A2::Pressure(void) {

	// Declare Output Variable
	float Value_;

	// Request WHO_AM_I Register
	Wire.beginTransmission(0b01100000);
	Wire.write(0b00001100);

	// Close I2C Connection
	int MPL3115A2_Sensor_Identification = Wire.endTransmission(false);

	// Control For Identifier Read Success
	if (MPL3115A2_Sensor_Identification != 0) return(-101);

	// Read Device Identifier Register
	Wire.requestFrom(0b01100000, 1);

	// Read WHO_AM_I Register
	uint8_t MPL3115A2_Device_Signiture = Wire.read();

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
		if (MPL3115A2_Sensor_CTRL_REG1_Register != 0) return(-102);
		
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
		if (MPL3115A2_Sensor_PT_DATA_CFG_Register != 0) return(-103);
		
		// ************************************************************
		// Read Sensor Data
		// ************************************************************
		
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
			if (MPL3115A2_Sensor_Pressure_Ready_Status != 0) return(-105);
			
			// Read Device Status Register
			Wire.requestFrom(0b01100000, 1);
			MPL3115A2_Read_Status = Wire.read();
			
			// Increase Counter
			Ready_Status_Try_Counter += 1;
			
			// Control for Wait Counter
			if (Ready_Status_Try_Counter > 50) return(-106);

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
		if (MPL3115A2_Sensor_Data_Read != 0) return(-107);
		
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
		Value_ = (1 * ((Measurement_Raw / 4.00 ) / 100)) + 0;

		// Read Delay
		delay(512);

	}

	if (Value_ <= 500 or Value_ >= 11000) return(-108);

	// End Function
	return(Value_);

}

// TSL2561 Functions
TSL2561::TSL2561(uint8_t _Measurement_Count, bool _Calibration_Enable) {

	// Set Measurement Count
	this->_Read_Count = _Measurement_Count;

	// Enable Calibration
	this->_Calibration = _Calibration_Enable;

}
float TSL2561::Light(void) {

	/******************************************************************************
	 *	Project		: TSL2561 Light Read Function
	 *	Developer	: Mehmet Gunce Akkoyun (akkoyun@me.com)
	 *	Revision	: 04.00.00
	 *	Release		: 04.11.2020
	 ******************************************************************************/
	
	// Define Sensor Settings
	int TSL2561_Integrate_Time 	= 1; // 13.7 ms - 0.034 Scale
	int TSL2561_Gain 			= 1; // 1x Gain
	
	// Declare Output Variable
	float Value_;

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
		if (TSL2561_Timing_Register_Write != 0) return(-101);
		
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
		if (TSL2561_Power_ON_Register_Write != 0) return(-102);
		
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
		long TSL2561_Ratio = (TSL2561_Channel_Ratio + 1) >> 1;
		
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
		
		// Round LSB (2^(LUX_SCALE-1))
		TSL2561_Lux_Temp += (1 << 13);
		
		// Strip Fff Fractional Portion
		Value_ = TSL2561_Lux_Temp >> 14;
		
		/****************************************
		 * Read Delay
		 ****************************************/
		
		// Delay
		if (TSL2561_Integrate_Time == 1) delay(14);
		if (TSL2561_Integrate_Time == 2) delay(102);
		if (TSL2561_Integrate_Time == 3) delay(403);
		
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
		if (TSL2561_Power_OFF_Register_Write != 0) return(-103);
		
		// Power Off Delay
		delay(50);

	}
	else {
		
		// End Function
		return(-104);
		
	}
	
	// End Function
	return(Value_);

}

// MCP3422 Functions
MCP3422::MCP3422(uint8_t _Channel, uint8_t _Measurement_Count, bool _Calibration_Enable) {

	// Set Measurement Count
	this->_Read_Count = _Measurement_Count;

	// Enable Calibration
	this->_Calibration = _Calibration_Enable;

	// Channel
	this->_Channel = _Channel;

}
float MCP3422::Pressure(void) {

	/******************************************************************************
	 *	Project		: MCP3422 2 Channel ADC Converter Read Function
	 *	Developer	: Mehmet Gunce Akkoyun (akkoyun@me.com)
	 *	Revision	: 02.00.00
	 ******************************************************************************/
	
	// Static Variables
	const uint8_t _Gain 		= 1;
	const uint8_t _Resolution 	= 12;
	const uint8_t _Mode 		= 2;
	const uint8_t _Sensor_Max 	= 10;

	// Set I2C Connection Close
	if (Wire.available()) {
		
		int I2C_Close = Wire.endTransmission();
		
		// Control For Close Success
		if (I2C_Close != 0) {
			
			// End Function
			return(-101);
			
		}
		
	}

	// Define Pressure Read Array
	float Pressure_RAW_Array[_Read_Count];

	// Start I2C
	Wire.beginTransmission(0x68);
	Wire.write(0x06);
	Wire.endTransmission(false);

	// Setting Register
	uint8_t Setting_Register = 0b00000000;

	// Gain Setting
	if (_Gain == 1) {
		Setting_Register &= 0b11111100;
	} // x1
	if (_Gain == 2) {
		Setting_Register &= 0b11111101;
		Setting_Register |= 0b00000001;
	} // x2
	if (_Gain == 4) {
		Setting_Register &= 0b11111110;
		Setting_Register |= 0b00000010;
	} // x4
	if (_Gain == 8) {
		Setting_Register |= 0b00000011;
	} // x8

	// Resolution Setting
	if (_Resolution == 12) {
		Setting_Register &= 0b11110011;
	} // 12 bit
	if (_Resolution == 14) {
		Setting_Register &= 0b11110111;
		Setting_Register |= 0b00000100;
	} // 14 bit
	if (_Resolution == 16) {
		Setting_Register &= 0b11111011;
		Setting_Register |= 0b00001000;
	} // 16 bit
	if (_Resolution == 18) {
		Setting_Register |= 0b00001100;
	} // 18 bit

	// Mode Setting
	if (_Mode == 1) {
		Setting_Register |= 0b00010000;
	} // Continious Mode
	if (_Mode == 2) {
		Setting_Register &= 0b11101111;
	} // One Shot Mode

	// Channel Setting
	if (_Channel == 1) {
		Setting_Register &= 0b10011111;
	} // Channel 1
	if (_Channel == 2) {
		Setting_Register &= 0b10111111;
		Setting_Register |= 0b00100000;
	} // Channel 2

	// New Conversation Setting
	Setting_Register |= 0b10000000;
	
	// Start ADC Conversion for Channel x
	Wire.beginTransmission(0x68);
	Wire.write(Setting_Register);
	int I2C_Start_Conversation = Wire.endTransmission(false);

	// Control For Close Success
	if (I2C_Start_Conversation != 0) {
		
		// End Function
		return(-102);
		
	}

	// Conversation Delay
	delay(1);

	// Read Loop For Read Count
	for (int Read_ID = 0; Read_ID < _Read_Count; Read_ID++) {

		// Define Data Variable
		uint8_t Pressure_Data[3];
		double Pressure_RAW = 0;
		uint16_t _Data_RAW = 0;

		// Read ADC
		do {

			// Read Pressure Data
			Wire.requestFrom(0x68, 3);

			// Read Data
			if(Wire.available() == 3) {
				
				// Read I2C Data
				Pressure_Data[0] = Wire.read();
				Pressure_Data[1] = Wire.read();
				Pressure_Data[2] = Wire.read();
				
				// Combine Read Bytes
				_Data_RAW = ((uint16_t)Pressure_Data[0] << 8) | (uint16_t)Pressure_Data[1];

			}

		} while((Pressure_Data[2] & 0x80) != 0x00);

		// Calculate Data
		for (uint8_t i = 0; i <= 10; i++) if (bitRead(_Data_RAW, i) == true) Pressure_RAW += pow(2, i);

		// Calculate Pressure
		if (_Resolution == 12) Pressure_RAW_Array[Read_ID] = Pressure_RAW / 2047;

	}

	// Construct Object
	Array_Stats<float> Data_Array(Pressure_RAW_Array, sizeof(Pressure_RAW_Array) / sizeof(Pressure_RAW_Array[0]));

	// Calculate Average
	float _Value = Data_Array.Average(Data_Array.Arithmetic_Avg);

	_Value = (_Value * 1.5304) - 1.3437;

	// End Function
	return(_Value);

}

// Analog Read Functions
Analog::Analog(uint8_t _Channel, uint8_t _Read_Count, bool _Calibration, float _Cal_a, float _Cal_b) {

	// Set Channel Variable
	_Channel &= 0b00000111;

	// Set Variables
	Measurement.Read_Count = _Read_Count;
	Measurement.Calibration = _Calibration;
	Measurement.Cal_a = _Cal_a;
	Measurement.Cal_b = _Cal_b;

	/*
		MUX3-0 
		------
		0-0-0-0 : ADC0
		0-0-0-1 : ADC1
		0-0-1-0 : ADC2
		0-0-1-1 : ADC3
		0-1-0-0 : ADC4
		0-1-0-1 : ADC5
		0-1-1-0 : ADC6
		0-1-1-1 : ADC7

		REFS1-0
		-------
		0-0 : AREF used as VRef and internal VRef is turned off.
		0-1 : AVCC with external capacitor at the AREF pin is used as VRef.
		1-0 : Reserved.
		1-1 : Internal referance voltage of 2v56 is used with an external capacitor at AREF pin for VRef.

		ADPS2-0
		-------
		0-0-0 : 2
		0-0-1 : 2
		0-1-0 : 4
		0-1-1 : 8
		1-0-0 : 16
		1-0-1 : 32
		1-1-0 : 64
		1-1-1 : 128
		7.372.800 Hz / Prescaler = 230.400 Hz --> Prescaler = 32

	*/

	// Set MUXx Bits
	ADMUX = (ADMUX & 0xF0) | _Channel;
	
	// Set REFSx Bits
	ADMUX = (ADMUX & 0x3F);
	ADMUX |= (1<<REFS0);

	// Set ADPSx (Prescaler)
	ADCSRA = (ADCSRA & 0xF8) | 0x05;

	// Set ADEN Bit (ADC Enable)
	ADCSRA |= (1<<ADEN);

}
double Analog::Read(void) {

	// Define Measurement Read Array
	double _Array[Measurement.Read_Count];

	// Read Loop For Read Count
	for (size_t Read_ID = 0; Read_ID < Measurement.Read_Count; Read_ID++) {

		// Start Measurement
		ADCSRA |= (1<<ADSC);

		// Wait While Measurement
		while(ADCSRA & (1 << ADIF));

		// Get Measurement
		uint16_t _Raw_Data = ADC;

		// Calculate Raw Pressure
		double _Pressure = ((float)10 * (float)_Raw_Data) / (float)1023;

		// Calibrate Measurement
		if (Measurement.Calibration) { 

			// Calibrate
			_Array[Read_ID] = (Measurement.Cal_a * _Pressure) + Measurement.Cal_b;

		} else {

			// Dont Calibrate
			_Array[Read_ID] = _Pressure;

		}

	}

	// Construct Object
	Array_Stats<double> Data_Array(_Array, Measurement.Read_Count);

	// Calculate Average
	double _Data = Data_Array.Average(Data_Array.Ext_RMS_Avg);

	// Set Statistical Parameter
	Standart_Deviation = Data_Array.Standard_Deviation();

	// Print Array
	//Data_Array.Array();

	// End Function
	return(_Data);

}

// 1903


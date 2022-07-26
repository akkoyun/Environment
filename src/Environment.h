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

// Include Statistical Library
#ifndef __Statistical__
	#include <Statistical.h>
#endif

/**
 * @brief AVR Analog Read Class
 * @version 01.00.00
 */
class Analog {

	private:

		// Analog Class Struct Definition
		struct Analog_Struct {

			/**
			 * @brief Measurement Read Count Variable (if not defined 1 measurement make).
			 */
			uint8_t Read_Count;

			/**
			 * @brief Measurement Calibration Enable Variable (if set true library make calibration).
			 */
			bool Calibration;

			/**
			 * @brief Calibration (aX+B) Gain Variable
			 */
			float Calibration_Gain;
			
			/**
			 * @brief Calibration (aX+B) Offset Variable
			 */
			float Calibration_Offset;

		} Measurement;

	public:

		// Statistical Parameters
		float Standard_Deviation;

		/**
		 * @brief Construct a new Analog object
		 * @param _Channel Analog Channel
		 * @param _Read_Count Measurement Count (default 1)
		 * @param _Calibration Measurement Calibration (default false)
		 * @param _Cal_a Calibration Gain (default 1)
		 * @param _Cal_b Calibration Offset (default 0)
		 */
		Analog(uint8_t _Channel, uint8_t _Read_Count = 1, bool _Calibration = false, float _Cal_a = 1, float _Cal_b = 0) {

			// Set Channel Variable
			_Channel &= 0b00000111;

			// Set Variables
			this->Measurement.Read_Count = _Read_Count;
			this->Measurement.Calibration = _Calibration;
			this->Measurement.Calibration_Gain = _Cal_a;
			this->Measurement.Calibration_Offset = _Cal_b;

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
				1-1 : Internal reference voltage of 2v56 is used with an external capacitor at AREF pin for VRef.

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

		/**
		 * @brief Analog Read Function
		 * @return float Measurement
		 * @version 01.00.00
		 */
		float Read(void) {

			// Define Measurement Read Array
			double _Array[this->Measurement.Read_Count];

			// Read Loop For Read Count
			for (size_t Read_ID = 0; Read_ID < this->Measurement.Read_Count; Read_ID++) {

				// Start Measurement
				ADCSRA |= (1<<ADSC);

				// Wait While Measurement
				while(ADCSRA & (1 << ADIF));

				// Get Measurement
				uint16_t _Raw_Data = ADC;

				// Calculate Raw Pressure
				double _Pressure = ((float)10 * (float)_Raw_Data) / (float)1023;

				// Calibrate Measurement
				if (this->Measurement.Calibration) { 

					// Calibrate
					_Array[Read_ID] = (this->Measurement.Calibration_Gain * _Pressure) + this->Measurement.Calibration_Offset;

				} else {

					// Dont Calibrate
					_Array[Read_ID] = _Pressure;

				}

			}

			// Construct Object
			Array_Stats<double> Data_Array(_Array, this->Measurement.Read_Count);

			// Calculate Average
			double _Data = Data_Array.Average(4);

			// Set Statistical Parameter
			this->Standard_Deviation = Data_Array.Standard_Deviation();

			// End Function
			return(_Data);
			
		}

};

/**
 * @brief HDC2010 TH Sensor Class
 * @version 01.00.00
 */
class HDC2010 : public I2C_Functions {

	private:

		/**
		 * @brief HDC2010 Sensor Variable Structure.
		 */
		struct HDC2010_Struct {

			/**
			 * @brief HDC2010 Sensor Address Variable.
			 */
			uint8_t TWI_Address 		= 0x40;

			/**
			 * @brief Sensor Mux Variable (if after a I2C multiplexer).
			 */
			bool Mux_Enable 			= false;

			/**
			 * @brief Sensor Mux Channel (if after a I2C multiplexer).
			 */
			uint8_t Mux_Channel 		= 0;

			/**
			 * @brief Measurement Read Count Variable (if not defined 1 measurement make).
			 */
			uint8_t Read_Count 			= 1;

			/**
			 * @brief Measurement Calibration Enable Variable (if set true library make calibration).
			 */
			bool Calibration 			= false;

			/**
			 * @brief Temperature Calibration (aX+B) Gain Variable
			 */
			float Calibration_T_Gain	= 1.0053;
			
			/**
			 * @brief Temperature Calibration (aX+B) Offset Variable
			 */
			float Calibration_T_Offset	= -0.4102;

			/**
			 * @brief Humidity Calibration (aX+B) Gain Variable
			 */
			float Calibration_H_Gain	= 0.9821;
			
			/**
			 * @brief Humidity Calibration (aX+B) Offset Variable
			 */
			float Calibration_H_Offset	= -0.3217;

		} Sensor;

	public:

		/**
		 * @brief Construct a new HDC2010 object
		 * @param _Multiplexer_Enable I2C Multiplexer Enable
		 * @param _Multiplexer_Channel I2C Multiplexer Channel
		 * @param _Measurement_Count Measurement Count
		 * @param _Calibration_Enable Calibration Enable
		 * @version 01.00.00
		 */
		HDC2010(bool _Multiplexer_Enable, uint8_t _Multiplexer_Channel, uint8_t _Measurement_Count = 1, bool _Calibration_Enable = false) : I2C_Functions(this->Sensor.TWI_Address, _Multiplexer_Enable, _Multiplexer_Channel) {

			// Set Measurement Count
			this->Sensor.Read_Count = _Measurement_Count;

			// Enable Calibration
			this->Sensor.Calibration = _Calibration_Enable;

			// Set Multiplexer Variables
			this->Sensor.Mux_Enable = _Multiplexer_Enable;
			this->Sensor.Mux_Channel = _Multiplexer_Channel;

		}

		/**
		 * @brief Calibration Parameters Set Function.
		 * @param _Measurement_Type Measurement Type
		 * 1 - Temperature
		 * 2 - Humidity
		 * @param _Gain Gain 
		 * @param _Offset Offset
		 */
		void Set_Calibration_Parameters(uint8_t _Measurement_Type, float _Gain, float _Offset) {

			// Set Temperature Calibration Parameters
			if (_Measurement_Type == 1) {

				// Set Gain
				this->Sensor.Calibration_T_Gain = _Gain;

				// Set Offset
				this->Sensor.Calibration_T_Offset = _Offset;

			}

			// Set Humidity Calibration Parameters
			if (_Measurement_Type == 2) {

				// Set Gain
				this->Sensor.Calibration_H_Gain = _Gain;

				// Set Offset
				this->Sensor.Calibration_H_Offset = _Offset;
				
			}

		}

		/**
		 * @brief Read Temperature Function
		 * @return float Temperature Measurement
		 * @version 01.00.00
		 */
		float Temperature(void) {

			// Read Register
			uint8_t HDC2010_Config_Read = Read_Register(0x0E);

			// Read Register
			uint8_t HDC2010_MeasurementConfig_Read = Read_Register(0x0F);

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
			Write_Register(0x0E, HDC2010_Config_Read, false);

			// Write Register
			Write_Register(0x0F, HDC2010_MeasurementConfig_Read, false);

			// Define Measurement Read Array
			float Measurement_Array[this->Sensor.Read_Count];

			// Read Loop For Read Count
			for (int Read_ID = 0; Read_ID < this->Sensor.Read_Count; Read_ID++) {

				// Define Variables
				uint8_t HDC2010_Data[2];

				// Read Delay
				delay(5);

				// Read Register
				HDC2010_Data[0] = Read_Register(0x00);
				HDC2010_Data[1] = Read_Register(0x01);

				// Combine Read Bytes
				uint16_t Measurement_Raw = ((uint16_t)(HDC2010_Data[1]) << 8 | (uint16_t)HDC2010_Data[0]);

				// Calculate Measurement
				Measurement_Array[Read_ID] = (float)Measurement_Raw * 165 / 65536 - 40;

			}

			// Construct Object
			Array_Stats<float> Data_Array(Measurement_Array, this->Sensor.Read_Count);

			// Calculate Average
			float Value_ = Data_Array.Average(Data_Array.Arithmetic_Avg);

			// Calibrate Data
			if (this->Sensor.Calibration) Value_ = (this->Sensor.Calibration_T_Gain * Value_) + this->Sensor.Calibration_T_Offset;

			// Control For Sensor Range
			if (Value_ < -40 or Value_ > 125) Value_ = -101;

			// End Function
			return(Value_);
			
		}

		/**
		 * @brief Read Humidity Function
		 * @return float Humidity Measurement
		 * @version 01.00.00
		 */
		float Humidity(void) {

			// Read Register
			uint8_t HDC2010_Config_Read = Read_Register(0x0E);

			// Read Register
			uint8_t HDC2010_MeasurementConfig_Read = Read_Register(0x0F);

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
			Write_Register(0x0E, HDC2010_Config_Read, false);

			// Write Register
			Write_Register(0x0F, HDC2010_MeasurementConfig_Read, false);

			// Define Measurement Read Array
			float Measurement_Array[this->Sensor.Read_Count];

			// Read Loop For Read Count
			for (int Read_ID = 0; Read_ID < this->Sensor.Read_Count; Read_ID++) {

				// Define Variables
				uint8_t HDC2010_Data[2];

				// Read Delay
				delay(5);

				// Read Register
				HDC2010_Data[0] = Read_Register(0x02);
				HDC2010_Data[1] = Read_Register(0x03);

				// Combine Read Bytes
				uint16_t Measurement_Raw = ((uint16_t)(HDC2010_Data[1]) << 8 | (uint16_t)HDC2010_Data[0]);

				// Calculate Measurement
				Measurement_Array[Read_ID] = (float)Measurement_Raw / 65536 * 100;

			}

			// Construct Object
			Array_Stats<float> Data_Array(Measurement_Array, this->Sensor.Read_Count);

			// Calculate Average
			float Value_ = Data_Array.Average(Data_Array.Arithmetic_Avg);

			// Calibrate Data
			if (this->Sensor.Calibration) Value_ = (this->Sensor.Calibration_H_Gain * Value_) + this->Sensor.Calibration_H_Offset;

			// Control For Sensor Range
			if (Value_ < 0 or Value_ > 100) Value_ = -101;

			// End Function
			return(Value_);
			
		}

};

/**
 * @brief MPL3115A2 Pressure Sensor Class
 * @version 01.00.00
 */
class MPL3115A2 : public I2C_Functions {

	private:

		/**
		 * @brief MPL3115A2 Sensor Variable Structure.
		 */
		struct MPL3115A2_Struct {

			/**
			 * @brief MPL3115A2 Sensor Address Variable.
			 */
			uint8_t TWI_Address 		= 0x60;

			/**
			 * @brief Sensor Mux Variable (if after a I2C multiplexer).
			 */
			bool Mux_Enable 			= false;

			/**
			 * @brief Sensor Mux Channel (if after a I2C multiplexer).
			 */
			uint8_t Mux_Channel 		= 0;

			/**
			 * @brief Measurement Read Count Variable (if not defined 1 measurement make).
			 */
			uint8_t Read_Count 			= 1;

			/**
			 * @brief Measurement Calibration Enable Variable (if set true library make calibration).
			 */
			bool Calibration 			= false;

			/**
			 * @brief Pressure Calibration (aX+B) Gain Variable
			 */
			float Calibration_P_Gain	= 1;
			
			/**
			 * @brief Pressure Calibration (aX+B) Offset Variable
			 */
			float Calibration_P_Offset	= 0;

		} Sensor;

	public:

		/**
		 * @brief Construct a new MPL3115A2 object
		 * @param _Multiplexer_Enable I2C Multiplexer Enable
		 * @param _Multiplexer_Channel I2C Multiplexer Channel
		 * @param _Measurement_Count Measurement Count
		 * @param _Calibration_Enable Calibration Enable
		 * @version 01.00.00
		 */
		MPL3115A2(bool _Multiplexer_Enable, uint8_t _Multiplexer_Channel, uint8_t _Measurement_Count = 1, bool _Calibration_Enable = false) : I2C_Functions(this->Sensor.TWI_Address, _Multiplexer_Enable, _Multiplexer_Channel) {

			// Set Measurement Count
			this->Sensor.Read_Count = _Measurement_Count;

			// Enable Calibration
			this->Sensor.Calibration = _Calibration_Enable;

			// Set Multiplexer Variables
			this->Sensor.Mux_Enable = _Multiplexer_Enable;
			this->Sensor.Mux_Channel = _Multiplexer_Channel;

		}

		/**
		 * @brief Calibration Parameters Set Function.
		 * @param _Gain Gain 
		 * @param _Offset Offset
		 */
		void Set_Calibration_Parameters(float _Gain, float _Offset) {

			// Set Gain
			this->Sensor.Calibration_P_Gain = _Gain;

			// Set Offset
			this->Sensor.Calibration_P_Offset = _Offset;

		}

		/**
		 * @brief Read Pressure Function
		 * @return float Pressure Measurement
		 * @version 01.00.00
		 */
		float Pressure(void) {

			// Read Register
			uint8_t MPL3115A2_Device_Signiture = Read_Register(0x0C);

			// Control for Device Identifier
			if (MPL3115A2_Device_Signiture == 0xC4) {

				// Set CTRL_REG1 Register
				Write_Register(0x26, 0x39, false);

				// Set PT_DATA_CFG Register
				Write_Register(0x13, 0x07, false);

				// Define Variables
				uint8_t MPL3115A2_Read_Status = 0;
				uint8_t Ready_Status_Try_Counter = 0;

				// Wait for Measurement Complete
				while ((MPL3115A2_Read_Status & 0b00000100) != 0b00000100) {
					
					//  Request Pressure Ready Status
					MPL3115A2_Read_Status = Read_Register(0x00);

					// Increase Counter
					Ready_Status_Try_Counter += 1;
					
					// Control for Wait Counter
					if (Ready_Status_Try_Counter > 50) return(0);

					// Ready Status Wait Delay
					if ((MPL3115A2_Read_Status & 0b00000100) != 0b00000100) delay(50);
					
				}

				// Define Measurement Read Array
				float Measurement_Array[this->Sensor.Read_Count];

				// Read Loop For Read Count
				for (int Read_ID = 0; Read_ID < this->Sensor.Read_Count; Read_ID++) {

					// Define Variables
					uint8_t MPL3115A2_Data[3];

					// Read Delay
					delay(5);

					// Read Register
					Read_Multiple_Register(0x01, MPL3115A2_Data, 3, false);

					// Define Variables
					uint32_t Measurement_Raw = 0;

					// Combine Read Bytes
					Measurement_Raw = MPL3115A2_Data[0];
					Measurement_Raw <<= 8;
					Measurement_Raw |= MPL3115A2_Data[1];
					Measurement_Raw <<= 8;
					Measurement_Raw |= MPL3115A2_Data[2];
					Measurement_Raw >>= 4;
					
					// Calculate Measurement
					Measurement_Array[Read_ID] = (this->Sensor.Calibration_P_Gain * ((Measurement_Raw / 4.00 ) / 100)) + this->Sensor.Calibration_P_Offset;

					// Read Delay
					if (this->Sensor.Read_Count != 1) delay(512);

				}

				// Construct Object
				Array_Stats<float> Data_Array(Measurement_Array, this->Sensor.Read_Count);

				// Calculate Average
				float Value_ = Data_Array.Average(Data_Array.Arithmetic_Avg);

				// Control For Sensor Range
				if (Value_ <= 500 or Value_ >= 11000) Value_ = 0;

				// End Function
				return(Value_);

			}

			// End Function
			return(0);

};

};

#endif /* defined(__Environment__) */

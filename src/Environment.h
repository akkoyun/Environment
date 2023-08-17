#ifndef __Environment__
#define __Environment__

	// Include Arduino Library
	#ifndef __Arduino__
		#include <Arduino.h>
	#endif

	// Include I2C Functions Library
	#ifndef __I2C_Functions__
		#include <I2C_Functions.h>
	#endif

	// Include Statistical Library
	#ifndef __Statistical__
		#include <Statistical.h>
	#endif

	// AVR Analog Read Class
	class Analog {

		// Private Context
		private:

			// Analog Class Struct Definition
			struct Analog_Struct {

				// Set Scale Variable
				float Scale 			= 10;

				// Analog Calibration Structure.
				struct Calibration_Struct{

					// Measurement Calibration Enable Variable (if set true library make calibration).
					bool Enable 			= false;

					// Analog Calibration (aX+B) Gain Variable
					float Gain 				= 1;

					// Analog Calibration (aX+B) Offset Variable
					float Offset 			= 0;

				} Calibration;

			} Measurement;

		// Public Context
		public:

			// Construct a new Analog object
			Analog(uint8_t _Channel) {

				// Set Channel Variable
				_Channel &= 0b00000111;

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

			// Calibration Parameters Set Function.
			void Set_Calibration_Parameters(const float _Gain, const float _Offset, const float _Scale) {

				// Set Gain
				this->Measurement.Calibration.Gain = _Gain;

				// Set Offset
				this->Measurement.Calibration.Offset = _Offset;

				// Enable Calibration
				if (this->Measurement.Calibration.Gain != 1 || this->Measurement.Calibration.Offset != 0) this->Measurement.Calibration.Enable = true;

				// Set Scale
				this->Measurement.Scale = _Scale;

			}

			// Analog Read Function
			float Read(const uint8_t _Read_Count) {

				// Define Measurement Read Array
				double _Measurement_Array[_Read_Count];

				// Read Loop For Read Count
				for (uint8_t _Read_ID = 0; _Read_ID < _Read_Count; _Read_ID++) {

					// Start Measurement
					ADCSRA |= (1<<ADSC);

					// Wait While Measurement
					while(ADCSRA & (1 << ADIF));

					// Get Measurement
					uint16_t _Raw_Data = ADC;

					// Calculate Raw Pressure
					double _RAW_Measurement = ((float)this->Measurement.Scale * (float)_Raw_Data) / (float)1023;

					// Calibrate Measurement
					if (this->Measurement.Calibration.Enable) { 

						// Calibrate
						_Measurement_Array[_Read_ID] = (this->Measurement.Calibration.Gain * _RAW_Measurement) + this->Measurement.Calibration.Offset;

					} else {

						// Dont Calibrate
						_Measurement_Array[_Read_ID] = _RAW_Measurement;

					}

				}

				// Control for Read Count
				if (_Read_Count > 1) {
					
					// Construct Object
					Array_Stats<double> Data_Array(_Measurement_Array, _Read_Count);

					// Calculate Average
					double _Value = Data_Array.Sigma_Average(1);

					// End Function
					return(_Value);

				} else {

					// End Function
					return(_Measurement_Array[0]);

				} 
				
			}

	};

	// HDC2010 TH Sensor Class
	class HDC2010 : private I2C_Functions {

		// Private Context
		private:

			// HDC2010 Sensor Variable Structure.
			struct HDC2010_Struct {

				// Sensor Address Variable.
				uint8_t TWI_Address 		= 0x40;

				// Multiplexer Structure.
				struct Multiplexer_Struct{

					// Multiplexer Enable Variable (if set true library make multiplexer).
					bool Enable 			= false;

					// Multiplexer Channel Variable (if not defined 0 channel make).
					uint8_t Channel 		= 0;

				} Multiplexer;

				// Pressure Calibration Structure.
				struct Calibration_Struct{

					// Measurement Calibration Enable Variable (if set true library make calibration).
					bool Enable_T 			= false;
					bool Enable_H 			= false;

					// Temperature Calibration (aX+B) Gain Variable
					float Gain_T 			= 1;

					// Temperature Calibration (aX+B) Offset Variable
					float Offset_T 			= 0;

					// Humidity Calibration (aX+B) Gain Variable
					float Gain_H 			= 1;

					// Humidity Calibration (aX+B) Offset Variable
					float Offset_H 			= 0;

				} Calibration;

			} Sensor;

		// Public Context
		public:

			// Construct a new HDC2010 object
			HDC2010(const bool _Multiplexer_Enable = false, const uint8_t _Multiplexer_Channel = 0) : I2C_Functions(__I2C_Addr_HDC2010__, _Multiplexer_Enable, _Multiplexer_Channel) {

				// Set Multiplexer Variables
				this->Sensor.Multiplexer.Enable = _Multiplexer_Enable;
				this->Sensor.Multiplexer.Channel = _Multiplexer_Channel;

				// Set Multiplexer Channel
				if (this->Sensor.Multiplexer.Enable) I2C_Functions::Set_Multiplexer();

			}

			// Calibration Parameters Set Function.
			void Set_Calibration_Parameters(uint8_t _Measurement_Type, float _Gain, float _Offset) {

				// Set Calibration Parameters
				if (_Measurement_Type == 1) {

					// Set Temperature Gain
					this->Sensor.Calibration.Gain_T = _Gain;

					// Set Temperature Offset
					this->Sensor.Calibration.Offset_T = _Offset;

					// Enable Calibration
					if (this->Sensor.Calibration.Gain_T != 1 || this->Sensor.Calibration.Offset_T != 0) this->Sensor.Calibration.Enable_T = true;

				} else if (_Measurement_Type == 2) {

					// Set Humidity Gain
					this->Sensor.Calibration.Gain_H = _Gain;

					// Set Humidity Offset
					this->Sensor.Calibration.Offset_H = _Offset;

					// Enable Calibration
					if (this->Sensor.Calibration.Gain_H != 1 || this->Sensor.Calibration.Offset_H != 0) this->Sensor.Calibration.Enable_H = true;

				} else {

					// Reset Calibration Parameters
					this->Sensor.Calibration.Gain_T = 1;
					this->Sensor.Calibration.Offset_T = 0;
					this->Sensor.Calibration.Gain_H = 1;
					this->Sensor.Calibration.Offset_H = 0;

				}

			}

			// Read Temperature Function
			float Temperature(const uint8_t _Measurement_Count = 1) {

				// Read Register
				uint8_t _HDC2010_Config_Read = I2C_Functions::Read_Register(0x0E);

				// Read Register
				uint8_t _HDC2010_MeasurementConfig_Read = I2C_Functions::Read_Register(0x0F);

				// Set Measurement Rate
				_HDC2010_Config_Read &= 0x8F;

				// Set Measurement Mode
				_HDC2010_MeasurementConfig_Read &= 0xFC;
				_HDC2010_MeasurementConfig_Read |= 0x02;

				// Set Temperature Resolution (9 bit)
				_HDC2010_MeasurementConfig_Read &= 0xBF;
				_HDC2010_MeasurementConfig_Read |= 0x80;

				// Set Humidity Resolution (9 bit)
				_HDC2010_MeasurementConfig_Read &= 0xEF;
				_HDC2010_MeasurementConfig_Read |= 0x20;

				// Trigger Measurement
				_HDC2010_MeasurementConfig_Read |= 0x01;

				// Write Register
				I2C_Functions::Write_Register(0x0E, _HDC2010_Config_Read, false);

				// Write Register
				I2C_Functions::Write_Register(0x0F, _HDC2010_MeasurementConfig_Read, false);

				// Define Measurement Read Array
				float _Measurement_Array[_Measurement_Count];

				// Read Loop For Read Count
				for (uint8_t _Read_ID = 0; _Read_ID < _Measurement_Count; _Read_ID++) {

					// Define Variables
					uint8_t _HDC2010_Data[2];

					// Read Register
					_HDC2010_Data[0] = I2C_Functions::Read_Register(0x00);
					_HDC2010_Data[1] = I2C_Functions::Read_Register(0x01);

					// Read Delay
					delay(5);

					// Combine Read Bytes
					uint16_t _Measurement_Raw = ((uint16_t)(_HDC2010_Data[1]) << 8 | (uint16_t)_HDC2010_Data[0]);

					// Control for Calibration
					if (this->Sensor.Calibration.Enable_T) {

						// Calculate Measurement
						_Measurement_Array[_Read_ID] = (this->Sensor.Calibration.Gain_T * ((float)_Measurement_Raw * 165.0 / 65536.0 - 40.0)) + this->Sensor.Calibration.Offset_T;

					} else {

						// Calculate Measurement
						_Measurement_Array[_Read_ID] = (_Measurement_Raw / 4.00 ) / 100;

					}

				}

				// Control for Read Count
				if (_Measurement_Count > 1) {
					
					// Construct Object
					Array_Stats<float> Data_Array(_Measurement_Array, _Measurement_Count);

					// Calculate Average
					float _Value = Data_Array.Average(_Arithmetic_Average_);

					// Control For Sensor Range
					if (_Value < -40 or _Value > 125) return(-101);

					// End Function
					return(_Value);

				} else {

					// Control For Sensor Range
					if (_Measurement_Array[0] < -40 or _Measurement_Array[0] > 125) return(-101);

					// End Function
					return(_Measurement_Array[0]);

				} 
				
			}

			// Read Humidity Function
			float Humidity(const uint8_t _Measurement_Count = 1) {

				// Read Register
				uint8_t _HDC2010_Config_Read = I2C_Functions::Read_Register(0x0E);

				// Read Register
				uint8_t _HDC2010_MeasurementConfig_Read = I2C_Functions::Read_Register(0x0F);

				// Set Measurement Rate
				_HDC2010_Config_Read &= 0xDF;
				_HDC2010_Config_Read |= 0x50;

				// Set Measurement Mode
				_HDC2010_MeasurementConfig_Read &= 0xFD;
				_HDC2010_MeasurementConfig_Read |= 0x04;

				// Set Temperature Resolution (9 bit)
				_HDC2010_MeasurementConfig_Read &= 0xBF;
				_HDC2010_MeasurementConfig_Read |= 0x80;

				// Set Humidity Resolution (9 bit)
				_HDC2010_MeasurementConfig_Read &= 0xEF;
				_HDC2010_MeasurementConfig_Read |= 0x20;

				// Trigger Measurement
				_HDC2010_MeasurementConfig_Read |= 0x01;

				// Write Register
				I2C_Functions::Write_Register(0x0E, _HDC2010_Config_Read, false);

				// Write Register
				I2C_Functions::Write_Register(0x0F, _HDC2010_MeasurementConfig_Read, false);

				// Define Measurement Read Array
				float _Measurement_Array[_Measurement_Count];

				// Read Loop For Read Count
				for (uint8_t _Read_ID = 0; _Read_ID < _Measurement_Count; _Read_ID++) {

					// Define Variables
					uint8_t _HDC2010_Data[2];

					// Read Delay
					delay(5);

					// Read Register
					_HDC2010_Data[0] = I2C_Functions::Read_Register(0x02);
					_HDC2010_Data[1] = I2C_Functions::Read_Register(0x03);

					// Combine Read Bytes
					uint16_t _Measurement_Raw = ((uint16_t)(_HDC2010_Data[1]) << 8 | (uint16_t)_HDC2010_Data[0]);

					// Control for Calibration
					if (this->Sensor.Calibration.Enable_H) {

						// Calculate Measurement
						_Measurement_Array[_Read_ID] = (this->Sensor.Calibration.Gain_H * ((float)_Measurement_Raw / 65536 * 100)) + this->Sensor.Calibration.Offset_H;

					} else {

						// Calculate Measurement
						_Measurement_Array[_Read_ID] = (_Measurement_Raw / 4.00 ) / 100;

					}

				}

				// Control for Read Count
				if (_Measurement_Count > 1) {
					
					// Construct Object
					Array_Stats<float> Data_Array(_Measurement_Array, _Measurement_Count);

					// Calculate Average
					float _Value = Data_Array.Average(_Arithmetic_Average_);

					// Control For Sensor Range
					if (_Value < 0 or _Value > 100) return(-101);

					// End Function
					return(_Value);

				} else {

					// Control For Sensor Range
					if (_Measurement_Array[0] < 0 or _Measurement_Array[0] > 100) return(-101);

					// End Function
					return(_Measurement_Array[0]);

				} 
				
			}

	};

	// MPL3115A2 Pressure Sensor Class
	class MPL3115A2 : private I2C_Functions {

		// Private Context
		private:

			// MPL3115A2 Sensor Variable Structure.
			struct MPL3115A2_Struct {

				// MPL3115A2 Sensor Address Variable.
				uint8_t TWI_Address 		= 0x60;

				// Multiplexer Structure.
				struct Multiplexer_Struct{

					// Multiplexer Enable Variable (if set true library make multiplexer).
					bool Enable 			= false;

					// Multiplexer Channel Variable (if not defined 0 channel make).
					uint8_t Channel 		= 0;

				} Multiplexer;

				// Pressure Calibration Structure.
				struct Calibration_Struct{

					// Measurement Calibration Enable Variable (if set true library make calibration).
					bool Enable 			= false;

					// Pressure Calibration (aX+B) Gain Variable
					float Gain 				= 1;

					// Pressure Calibration (aX+B) Offset Variable
					float Offset 			= 0;

				} Calibration;

			} Sensor;

		// Public Context
		public:

			// Construct a new MPL3115A2 object
			MPL3115A2(const bool _Multiplexer_Enable = false, const uint8_t _Multiplexer_Channel = 0) : I2C_Functions(__I2C_Addr_MPL3115A2__, _Multiplexer_Enable, _Multiplexer_Channel) {

				// Set Multiplexer Variables
				this->Sensor.Multiplexer.Enable = _Multiplexer_Enable;
				this->Sensor.Multiplexer.Channel = _Multiplexer_Channel;

				// Set Multiplexer Channel
				if (this->Sensor.Multiplexer.Enable) I2C_Functions::Set_Multiplexer();

			}

			// Calibration Parameters Set Function.
			void Set_Calibration_Parameters(const float _Gain, const float _Offset) {

				// Set Gain
				this->Sensor.Calibration.Gain = _Gain;

				// Set Offset
				this->Sensor.Calibration.Offset = _Offset;

				// Enable Calibration
				if (this->Sensor.Calibration.Gain != 1 || this->Sensor.Calibration.Offset != 0) this->Sensor.Calibration.Enable = true;

			}

			// Read Pressure Function
			float Pressure(const uint8_t _Measurement_Count = 1) {

				// Read Register
				uint8_t _MPL3115A2_Device_Signiture = I2C_Functions::Read_Register(0x0C);

				// Control for Device Identifier
				if (_MPL3115A2_Device_Signiture == 0xC4) {

					// Set CTRL_REG1 Register
					I2C_Functions::Write_Register(0x26, 0x39, false);

					// Set PT_DATA_CFG Register
					I2C_Functions::Write_Register(0x13, 0x07, false);

					// Define Variables
					uint8_t _MPL3115A2_Read_Status = 0;
					uint8_t _Ready_Status_Try_Counter = 0;

					// Wait for Measurement Complete
					while ((_MPL3115A2_Read_Status & 0b00000100) != 0b00000100) {
						
						//  Request Pressure Ready Status
						_MPL3115A2_Read_Status = I2C_Functions::Read_Register(0x00);

						// Increase Counter
						_Ready_Status_Try_Counter += 1;
						
						// Control for Wait Counter
						if (_Ready_Status_Try_Counter > 50) return(-102);

						// Ready Status Wait Delay
						if ((_MPL3115A2_Read_Status & 0b00000100) != 0b00000100) delay(50);
						
					}

					// Define Measurement Read Array
					float _Measurement_Array[_Measurement_Count];

					// Read Loop For Read Count
					for (uint8_t _Read_ID = 0; _Read_ID < _Measurement_Count; _Read_ID++) {

						// Define Variables
						uint8_t _MPL3115A2_Data[3];

						// Read Delay
						delay(5);

						// Read Register
						I2C_Functions::Read_Multiple_Register(0x01, _MPL3115A2_Data, 3, false);

						// Define Variables
						uint32_t _Measurement_Raw = 0;

						// Combine Read Bytes
						_Measurement_Raw |= (uint32_t)_MPL3115A2_Data[0] << 16;
						_Measurement_Raw |= (uint32_t)_MPL3115A2_Data[1] << 8;
						_Measurement_Raw |= (uint32_t)_MPL3115A2_Data[2];
						_Measurement_Raw >>= 4;
					
						// Control for Calibration
						if (this->Sensor.Calibration.Enable) {

							// Calculate Measurement
							_Measurement_Array[_Read_ID] = (this->Sensor.Calibration.Gain * ((_Measurement_Raw / 4.00 ) / 100)) + this->Sensor.Calibration.Offset;

						} else {

							// Calculate Measurement
							_Measurement_Array[_Read_ID] = (_Measurement_Raw / 4.00 ) / 100;

						}

						// Read Delay
						if (_Measurement_Count != 1) delay(512);

					}

					// Control for Read Count
					if (_Measurement_Count > 1) {
						
						// Construct Object
						Array_Stats<float> _Data_Array(_Measurement_Array, _Measurement_Count);

						// Calculate Average
						float _Value = _Data_Array.Average(_Arithmetic_Average_);

						// Control For Sensor Range
						if (_Value <= 500 or _Value >= 11000) return(-103);

						// End Function
						return(_Value);

					} else {

						// Control For Sensor Range
						if (_Measurement_Array[0] <= 500 or _Measurement_Array[0] >= 11000) return(-103);

						// End Function
						return(_Measurement_Array[0]);

					} 

				} 

				// End Function
				return(-101);

			};

	};

	// TSL2561 Light Sensor Class
	class TSL2561 : private I2C_Functions {

		private:

			/**
			 * @brief TSL2561 Sensor Variable Structure.
			 */
			struct TSL2561_Struct {

				/**
				 * @brief MPL3115A2 Sensor Address Variable.
				 */
				uint8_t TWI_Address 				= 0x39;

				/**
				 * @brief Sensor Mux Variable (if after a I2C multiplexer).
				 */
				bool Mux_Enable 					= false;

				/**
				 * @brief Sensor Mux Channel (if after a I2C multiplexer).
				 */
				uint8_t Mux_Channel 				= 0;

				/**
				 * @brief Measurement Read Count Variable (if not defined 1 measurement make).
				 */
				uint8_t Read_Count 					= 1;

				/**
				 * @brief Measurement Calibration Enable Variable (if set true library make calibration).
				 */
				bool Calibration 					= false;

				/**
				 * @brief Pressure Calibration (aX+B) Gain Variable
				 */
				float Calibration_L_Gain			= 1;
				
				/**
				 * @brief Pressure Calibration (aX+B) Offset Variable
				 */
				float Calibration_L_Offset			= 0;

				/**
				 * @brief Get Sensor Serial ID
				 * @version 01.00.00
				 */
				uint8_t TSL2561_ID 					= 0;

				/**
				 * @brief Sensor Gain Variable
				 */
				bool TSL2561_Gain 					= false;

				/**
				 * @brief Sensor Integration Time Variable
				 */
				uint16_t TSL2561_Integration_Time 	= 14;

				/**
				 * @brief Sensor Channel 0 Value
				 */
				uint16_t TSL2561_Data_0 			= 0;

				/**
				 * @brief Sensor Channel 1 Value
				 */
				uint16_t TSL2561_Data_1 			= 0;

				/**
				 * @brief Sensor LUX Value
				 */
				float TSL2561_Lux 					= 0;


			} Sensor;

			/**
			 * @brief Set TSL2561 Power Up
			 * @version 01.00.00
			 */
			void Set_Power_Up(void) {

				// Set Power On Register
				Write_Register(0x80, 0x03, false);

			}

			/**
			 * @brief Set TSL2561 Power Down
			 * @version 01.00.00
			 */
			void Set_Power_Down(void) {

				// Set Power On Register
				Write_Register(0x80, 0x00, false);

			}

			/**
			 * @brief Set Gain and Timing Registers
			 * @version 01.00.00
			 * @param _Gain 
			 * False - device is set to low gain (1X)
			 * True - device is set to high gain (16X)
			 * @param _Time 
			 * 0 - integration will be 13.7ms
			 * 1 - integration will be 101ms
			 * 2 - integration will be 402ms
			 * 3 - use manual start / stop
			 */
			void Set_Timing(const bool _Gain, const uint8_t _Time) {

				// Read Timing Register
				uint8_t TSL2561_Timing_Register  = Read_Register(0x81);

				// Set Integrate Time Bit Values
				if (_Time == 0) {
					
					// Set Bit 0 LOW
					TSL2561_Timing_Register &= 0b11111110;
					
					// Set Bit 1 LOW
					TSL2561_Timing_Register &= 0b11111101;
					
					// Set Sensor Variable
					this->Sensor.TSL2561_Integration_Time = 14;

				} // 0.034 Scale - 13.7 ms
				if (_Time == 1) {
					
					// Set Bit 0 HIGH
					TSL2561_Timing_Register |= 0b00000001;
					
					// Set Bit 1 LOW
					TSL2561_Timing_Register &= 0b11111101;

					// Set Sensor Variable
					this->Sensor.TSL2561_Integration_Time = 101;

				} // 0.252 Scale - 101 ms
				if (_Time == 2) {
					
					// Set Bit 0 LOW
					TSL2561_Timing_Register &= 0b11111110;
					
					// Set Bit 1 HIGH
					TSL2561_Timing_Register |= 0b00000010;

					// Set Sensor Variable
					this->Sensor.TSL2561_Integration_Time = 402;

				} // 1.000 Scale - 402 ms

				// Set Gain Bit Values
				if (!_Gain) {
					
					// Set Bit 4 LOW
					TSL2561_Timing_Register &= 0b11101111;

					// Set Sensor Variable
					this->Sensor.TSL2561_Gain = false;

				} // 0 - Low Gain (1x)
				if (_Gain) {
					
					// Set Bit 4 HIGH
					TSL2561_Timing_Register |= 0b00010000;

					// Set Sensor Variable
					this->Sensor.TSL2561_Gain = true;

				} // 1 - High Gain (16x)

				// Write Timing Register
				Write_Register(0x81, TSL2561_Timing_Register, false);

				// Delay
				delay(50);

			}

			/**
			 * @brief Get TSL2561 Channel Data
			 * @version 01.00.00
			 * @param Data0 Channel 0 Data
			 * @param Data1 Channel 1 Data
			 */
			void Get_Data(void) {

				// Combine Read Bytes
				this->Sensor.TSL2561_Data_0 = (Read_Register(0x8D) << 8) | Read_Register(0x8C);

				// Combine Read Bytes
				this->Sensor.TSL2561_Data_1 = (Read_Register(0x8F) << 8) | Read_Register(0x8E);

			}

			/**
			 * @brief Calculate Lux Value
			 * @version 01.00.00
			 * @return true 
			 * @return false 
			 */
			bool Calculate_LUX(void) {

				// Declare Variables
				double Ratio, D0, D1;

				// Control for Channel Error
				if ((this->Sensor.TSL2561_Data_0 == 0xFFFF) || (this->Sensor.TSL2561_Data_1 == 0xFFFF)) {

					// Set Variable
					this->Sensor.TSL2561_Lux = 0;

					// End Function
					return(false);

				}

				// Set Variable
				D0 = this->Sensor.TSL2561_Data_0;
				D1 = this->Sensor.TSL2561_Data_1;

				// Calculate Ratio
				Ratio = D1 / D0;

				// Normalize for integration time
				D0 *= (402.0 / this->Sensor.TSL2561_Integration_Time);
				D1 *= (402.0 / this->Sensor.TSL2561_Integration_Time);

				// Normalize for gain
				if (!this->Sensor.TSL2561_Gain) {D0 *= 16; D1 *= 16;}

				// Calculate Lux Value
				if (Ratio < 0.50) {this->Sensor.TSL2561_Lux = (0.0304 * D0) - (0.062 * D1 * pow(Ratio,1.4)); return(true);}
				if (Ratio < 0.61) {this->Sensor.TSL2561_Lux = (0.0224 * D0) - (0.031 * D1); return(true);}
				if (Ratio < 0.80) {this->Sensor.TSL2561_Lux = (0.0128 * D0) - (0.0153 * D1); return(true);}
				if (Ratio < 1.30) {this->Sensor.TSL2561_Lux = (0.00146 * D0) - (0.00112 * D1); return(true);}

				// End Function
				return(false);

			}

			/**
			 * @brief Get TSL2561 Serial ID
			 * @version 01.00.00
			 */
			void Get_Serial_ID(void) {

				// Read Serial ID Register
				this->Sensor.TSL2561_ID = Read_Register(0x8A);

			}

		public:

			/**
			 * @brief Construct a new MPL3115A2 object
			 * @param _Multiplexer_Enable I2C Multiplexer Enable
			 * @param _Multiplexer_Channel I2C Multiplexer Channel
			 * @param _Measurement_Count Measurement Count
			 * @param _Calibration_Enable Calibration Enable
			 * @version 01.00.00
			 */
			TSL2561(bool _Multiplexer_Enable, uint8_t _Multiplexer_Channel, uint8_t _Measurement_Count = 1, bool _Calibration_Enable = false) : I2C_Functions(__I2C_Addr_TSL2561__, _Multiplexer_Enable, _Multiplexer_Channel) {

				// Set Measurement Count
				_Measurement_Count = _Measurement_Count;

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
				this->Sensor.Calibration_L_Gain = _Gain;

				// Set Offset
				this->Sensor.Calibration_L_Offset = _Offset;

			}

			/**
			 * @brief Read Light Function
			 * @return float Light Measurement
			 * @version 01.00.00
			 */
			float Light(void) {

				// Get Serial ID
				this->Get_Serial_ID();

				// Control for Device ID
				if (this->Sensor.TSL2561_ID == 0b01010000 or this->Sensor.TSL2561_ID == 0b11111111) {

					// Set Gain and Integration Time
					this->Set_Timing(this->Sensor.TSL2561_Gain, this->Sensor.TSL2561_Integration_Time);

					// Power On Sensor
					this->Set_Power_Up();

					// Define Measurement Read Array
					float Measurement_Array[this->Sensor.Read_Count];

					// Read Loop For Read Count
					for (int Read_ID = 0; Read_ID < this->Sensor.Read_Count; Read_ID++) {

						// Get Sensor Channel Data
						this->Get_Data();

						// Calculate LUX
						this->Calculate_LUX();

						// Set Variable
						Measurement_Array[Read_ID] = this->Sensor.TSL2561_Lux;

						// Read Delay
						delay(this->Sensor.TSL2561_Integration_Time);

					}

					// Power Down Sensor
					this->Set_Power_Down();

					// Power Off Delay
					delay(50);

					// Construct Object
					Array_Stats<float> Data_Array(Measurement_Array, this->Sensor.Read_Count);

					// Calculate Average
					float Value_ = Data_Array.Average(Data_Array.Arithmetic_Avg);

					// Calibrate Data
					if (this->Sensor.Calibration) Value_ = (this->Sensor.Calibration_L_Gain * Value_) + this->Sensor.Calibration_L_Offset;

					// End Function
					return(Value_);

				}
					
				// End Function
				return(0);

			}

	};

	// SDP810 Delta Pressure Sensor Class
	class SDP810 : private I2C_Functions {

		private:

			/**
			 * @brief SDP810 Sensor Variable Structure.
			 */
			struct SDP810_Struct {

				/**
				 * @brief MPL3115A2 Sensor Address Variable.
				 */
				uint8_t TWI_Address 		= 0x25;

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
				float Calibration_DP_Gain	= 1;
				
				/**
				 * @brief Pressure Calibration (aX+B) Offset Variable
				 */
				float Calibration_DP_Offset	= 0;

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
			SDP810(bool _Multiplexer_Enable, uint8_t _Multiplexer_Channel, uint8_t _Measurement_Count = 1, bool _Calibration_Enable = false) : I2C_Functions(__I2C_Addr_SDP810__, _Multiplexer_Enable, _Multiplexer_Channel) {

				// Set Measurement Count
				this->Sensor.Read_Count = _Measurement_Count;

				// Enable Calibration
				this->Sensor.Calibration = _Calibration_Enable;

				// Set Multiplexer Variables
				this->Sensor.Mux_Enable = _Multiplexer_Enable;
				this->Sensor.Mux_Channel = _Multiplexer_Channel;

			}

	};

	// WeatherStat Class [B108AA]
	class B108AA_Environment : private HDC2010, private MPL3115A2 {

		// Public Context
		public:

			// WeatherStat Sensor Variable Structure.
			struct Weather_Struct {

				// HDC2010 Sensor Variable Structure.
				float Air_Temperature;
				float Air_Humidity;

				// MPL3115A2 Sensor Variable Structure.
				float Air_Pressure;

			} Sensor;

			// Class Constructor
			B108AA_Environment(void) : HDC2010(true, 2), MPL3115A2(true, 3) {

				// Set Calibration Parameters
				HDC2010::Set_Calibration_Parameters(1, 1, 0);
				HDC2010::Set_Calibration_Parameters(2, 1, 0);
				MPL3115A2::Set_Calibration_Parameters(1, 0);

			}

			// Read Sensor Function
			void Environment (void) {

				// Read HDC2010 Sensor
				this->Sensor.Air_Temperature = HDC2010::Temperature(10);
				this->Sensor.Air_Humidity = HDC2010::Humidity(10);

				// Read MPL3115A2 Sensor
				this->Sensor.Air_Pressure = MPL3115A2::Pressure(5);

			}

	};

#endif /* defined(__Environment__) */
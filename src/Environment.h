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

				// Pressure Calibration Structure.
				struct Calibration_Struct {

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

			// Enable the interrupt pin for threshold operation
			void Interrupt_Config(bool _DRDY = false, bool _TH = false, bool _TL = false, bool _HH = false, bool _HL = false) {

				// Read Register
				uint8_t _HDC2010_Interrupt_Config_Register = I2C_Functions::Read_Register(0x07);

				// Set Bits
				if (_DRDY) {bitSet(_HDC2010_Interrupt_Config_Register, 7);} else {bitClear(_HDC2010_Interrupt_Config_Register, 7);}
				if (_TH) {bitSet(_HDC2010_Interrupt_Config_Register, 6);} else {bitClear(_HDC2010_Interrupt_Config_Register, 6);}
				if (_TL) {bitSet(_HDC2010_Interrupt_Config_Register, 5);} else {bitClear(_HDC2010_Interrupt_Config_Register, 5);}
				if (_HH) {bitSet(_HDC2010_Interrupt_Config_Register, 4);} else {bitClear(_HDC2010_Interrupt_Config_Register, 4);}
				if (_HL) {bitSet(_HDC2010_Interrupt_Config_Register, 3);} else {bitClear(_HDC2010_Interrupt_Config_Register, 3);}

				// Write Register
				I2C_Functions::Write_Register(0x07, _HDC2010_Interrupt_Config_Register, true);
			
			}

			// Set Interrupt Mode Function
			void Interrupt_Mode(bool _Mode = false) {

				// Control for Mode
				if (_Mode) {

					// Set Bit
					I2C_Functions::Set_Register_Bit(0x0E, 0, true);	// Latched Mode

				} else {

					// Clear Bit
					I2C_Functions::Clear_Register_Bit(0x0E, 0, true);	// Transparent Mode

				}

			}

			// Set Interrupt Polarity Function
			void Interrupt_Polarity(bool _Polarity = true) {

				// Control for Polarity
				if (_Polarity) {

					// Set Bit
					I2C_Functions::Set_Register_Bit(0x0E, 1, true);	// Active High

				} else {

					// Clear Bit
					I2C_Functions::Clear_Register_Bit(0x0E, 1, true);	// Active Low

				}

			}

			// Set Rate Function
			void Set_Rate(uint8_t _Rate) {

				// Read Register
				uint8_t _HDC2010_MEASUREMENT_CONFIG_Read = I2C_Functions::Read_Register(0x0E);

				// Set Rate 
				if (_Rate == 0) {

					// Manual

					// Clear Bits
					_HDC2010_MEASUREMENT_CONFIG_Read &= 0x8F;

				} else if (_Rate == 1) {

					// 2 Minutes

					// Clear Bits
					_HDC2010_MEASUREMENT_CONFIG_Read &= 0x9F;

					// Set Bit
					_HDC2010_MEASUREMENT_CONFIG_Read |= 0x10;

				} else if (_Rate == 2) {

					// 1 Minute

					// Clear Bits
					_HDC2010_MEASUREMENT_CONFIG_Read &= 0xAF;

					// Set Bit
					_HDC2010_MEASUREMENT_CONFIG_Read |= 0x20;

				} else if (_Rate == 3) {

					// 10 Seconds

					// Clear Bits
					_HDC2010_MEASUREMENT_CONFIG_Read &= 0xBF;

					// Set Bit
					_HDC2010_MEASUREMENT_CONFIG_Read |= 0x30;

				} else if (_Rate == 4) {

					// 5 Seconds

					// Clear Bits
					_HDC2010_MEASUREMENT_CONFIG_Read &= 0xCF;

					// Set Bit
					_HDC2010_MEASUREMENT_CONFIG_Read |= 0x40;

				} else if (_Rate == 5) {

					// 1 Second

					// Clear Bits
					_HDC2010_MEASUREMENT_CONFIG_Read &= 0xDF;

					// Set Bit
					_HDC2010_MEASUREMENT_CONFIG_Read |= 0x50;

				} else if (_Rate == 6) {

					// 2 Hz

					// Clear Bits
					_HDC2010_MEASUREMENT_CONFIG_Read &= 0xEF;

					// Set Bit
					_HDC2010_MEASUREMENT_CONFIG_Read |= 0x60;

				} else if (_Rate == 7) {

					// 5 Hz

					// Clear Bits
					_HDC2010_MEASUREMENT_CONFIG_Read &= 0xFF;

					// Set Bit
					_HDC2010_MEASUREMENT_CONFIG_Read |= 0x70;

				} else {

					// Manual

					// Clear Bits
					_HDC2010_MEASUREMENT_CONFIG_Read &= 0x8F;

				}

				// Write Register
				I2C_Functions::Write_Register(0x0E, _HDC2010_MEASUREMENT_CONFIG_Read, true);

			}

			// Reset Sensor Function
			void Reset(void) {

				// Set Reset Bit
				I2C_Functions::Set_Register_Bit(0x0E, 7, true);

				// Delay
				delay(50);

			}

			// Trigger Measurement Function
			void Trigger_Measurement(void) {

				// Set Trigger Bit
				I2C_Functions::Set_Register_Bit(0x0F, 0, true);

			}

			// Set Measurement Mode Function
			void Set_Measurement_Mode(uint8_t _Mode) {

				// Read Register
				uint8_t _HDC2010_MeasurementConfig_Read = I2C_Functions::Read_Register(0x0F);

				// Set Mode 
				if (_Mode == 0) {

					// Temperature and Humidity

					// Clear Bits
					_HDC2010_MeasurementConfig_Read &= 0xF9;

				} else if (_Mode == 1) {

					// Temperature Only

					// Clear Bits
					_HDC2010_MeasurementConfig_Read &= 0xFC;

					// Set Bit
					_HDC2010_MeasurementConfig_Read |= 0x02;

				} else if (_Mode == 2) {

					// Humidity Only

					// Clear Bits
					_HDC2010_MeasurementConfig_Read &= 0xFD;

					// Set Bit
					_HDC2010_MeasurementConfig_Read |= 0x04;

				} else {

					// Temperature and Humidity

					// Clear Bits
					_HDC2010_MeasurementConfig_Read &= 0xF9;

				}

				// Write Register
				I2C_Functions::Write_Register(0x0F, _HDC2010_MeasurementConfig_Read, true);

			}

			// Set Temperature Resolution Function
			void Set_Temperature_Resolution(uint8_t _Resolution) {

				// Read Register
				uint8_t _HDC2010_MeasurementConfig_Read = I2C_Functions::Read_Register(0x0F);

				// Set Resolution 
				if (_Resolution == 14) {

					// 14 Bit

					// Clear Bits
					_HDC2010_MeasurementConfig_Read &= 0x3F;

				} else if (_Resolution == 11) {

					// 11 Bit

					// Clear Bits
					_HDC2010_MeasurementConfig_Read &= 0x7F;

					// Set Bit
					_HDC2010_MeasurementConfig_Read |= 0x40;

				} else if (_Resolution == 9) {

					// 9 Bit

					// Clear Bits
					_HDC2010_MeasurementConfig_Read &= 0xBF;

					// Set Bit
					_HDC2010_MeasurementConfig_Read |= 0x80;

				} else {

					// 14 Bit

					// Clear Bits
					_HDC2010_MeasurementConfig_Read &= 0x3F;

				}

				// Write Register
				I2C_Functions::Write_Register(0x0F, _HDC2010_MeasurementConfig_Read, true);

			}

			// Set Humidity Resolution Function
			void Set_Humidity_Resolution(uint8_t _Resolution) {

				// Read Register
				uint8_t _HDC2010_MeasurementConfig_Read = I2C_Functions::Read_Register(0x0F);

				// Set Resolution 
				if (_Resolution == 14) {

					// 14 Bit

					// Clear Bits
					_HDC2010_MeasurementConfig_Read &= 0xCF;

				} else if (_Resolution == 11) {

					// 11 Bit

					// Clear Bits
					_HDC2010_MeasurementConfig_Read &= 0xDF;

					// Set Bit
					_HDC2010_MeasurementConfig_Read |= 0x10;

				} else if (_Resolution == 9) {

					// 9 Bit

					// Clear Bits
					_HDC2010_MeasurementConfig_Read &= 0xEF;

					// Set Bit
					_HDC2010_MeasurementConfig_Read |= 0x20;

				} else {

					// 14 Bit

					// Clear Bits
					_HDC2010_MeasurementConfig_Read &= 0xCF;

				}

				// Write Register
				I2C_Functions::Write_Register(0x0F, _HDC2010_MeasurementConfig_Read, true);

			}

			// Enable / Disable Heater Function
			void Heater(bool _State = false) {

				// Control for State
				if (_State) {

					// Set Bit
					I2C_Functions::Set_Register_Bit(0x0E, 3, true);

				} else {

					// Clear Bit
					I2C_Functions::Clear_Register_Bit(0x0E, 3, true);

				}

			}

		// Public Context
		public:

			// Construct a new HDC2010 object
			HDC2010(const bool _Multiplexer_Enable = false, const uint8_t _Multiplexer_Channel = 0) : I2C_Functions(__I2C_Addr_HDC2010__, _Multiplexer_Enable, _Multiplexer_Channel) {

			}

			// HDC2010 Begin Function
			bool Begin(void) {

				// Start I2C
				I2C_Functions::Begin();

				// Control for Device
				if (I2C_Functions::Variables.Device.Detect) {

					// Reset Sensor
					this->Reset();

					// Set Measurement Limit
					this->Set_High_Temperature_Threshold(30);
					this->Set_Low_Temperature_Threshold(-20);
					this->Set_High_Humidity_Threshold(80);
					this->Set_Low_Humidity_Threshold(20);

					// Set Interrupt Pin
					this->Interrupt_Config(false, true, true, true, true);	// Enable Threshold Interrupts
					this->Interrupt_Mode(false);							// Transparent Mode
					this->Interrupt_Polarity(true);							// Active High
					this->Interrupt(true);									// Enable Interrupt

					// Configure Measurements
					this->Set_Measurement_Mode(0);
					this->Set_Rate(5);
					this->Set_Temperature_Resolution(14);
					this->Set_Humidity_Resolution(14);

					// Begin Measurement
					this->Trigger_Measurement();
					
					// End Function
					return(true);

				} else {

					// End Function
					return(false);

				}

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

			// Read Max Temperature Function
			float Read_Max_Temperature(void) {

				// Read Register
				uint8_t _HDC2010_TEMP_MAX_Read = I2C_Functions::Read_Register(0x05);

				// Calculate Temperature
				float _Temperature = (float)_HDC2010_TEMP_MAX_Read * 165 / 256 - 40;

				// End Function
				return(_Temperature);

			}

			// Read Max Humidity Function
			float Read_Max_Humidity(void) {

				// Read Register
				uint8_t _HDC2010_HUMID_MAX_Read = I2C_Functions::Read_Register(0x06);

				// Calculate Humidity
				float _Humidity = (float)_HDC2010_HUMID_MAX_Read / 256 * 100;

				// End Function
				return(_Humidity);

			}

			// Clear Max Temperature Function
			void Clear_Max_Temperature(void) {

				// Write Register
				I2C_Functions::Write_Register(0x05, 0x00, true);

			}

			// Clear Max Humidity Function
			void Clear_Max_Humidity(void) {

				// Write Register
				I2C_Functions::Write_Register(0x06, 0x00, true);

			}

			// Read Interrupt Status Function	
			uint8_t Read_Interrupt_Status(void) {

				// Read Register
				uint8_t _HDC2010_INT_DRDY_Read = I2C_Functions::Read_Register(0x04);

				// End Function
				return(_HDC2010_INT_DRDY_Read);

			}

			// Enable/Disable Interrupt Function
			void Interrupt(bool _State = false) {

				// Control for State
				if (_State) {

					// Set Bit
					I2C_Functions::Set_Register_Bit(0x0E, 2, true);

				} else {

					// Clear Bit
					I2C_Functions::Clear_Register_Bit(0x0E, 2, true);

				}

			}

			// Read Low Humidity Threshold Function
			float Read_Low_Humidity_Threshold(void) {

				// Read Register
				uint8_t _HDC2010_HUMID_THR_L_Read = I2C_Functions::Read_Register(0x0C);

				// Calculate Humidity
				float _Humidity = (float)_HDC2010_HUMID_THR_L_Read * 100 / 256;

				// End Function
				return(_Humidity);

			}

			// Read High Humidity Threshold Function
			float Read_High_Humidity_Threshold(void) {

				// Read Register
				uint8_t _HDC2010_HUMID_THR_H_Read = I2C_Functions::Read_Register(0x0D);

				// Calculate Humidity
				float _Humidity = (float)_HDC2010_HUMID_THR_H_Read * 100 / 256;

				// End Function
				return(_Humidity);

			}

			// Read Low Temperature Threshold Function
			float Read_Low_Temperature_Threshold(void) {

				// Read Register
				uint8_t _HDC2010_TEMP_THR_L_Read = I2C_Functions::Read_Register(0x0A);

				// Calculate Temperature
				float _Temperature = (float)_HDC2010_TEMP_THR_L_Read * 165 / 256 - 40;

				// End Function
				return(_Temperature);

			}

			// Read High Temperature Threshold Function
			float Read_High_Temperature_Threshold(void) {

				// Read Register
				uint8_t _HDC2010_TEMP_THR_H_Read = I2C_Functions::Read_Register(0x0B);

				// Calculate Temperature
				float _Temperature = (float)_HDC2010_TEMP_THR_H_Read * 165 / 256 - 40;

				// End Function
				return(_Temperature);

			}

			// Set Low Temperature Threshold Function
			void Set_Low_Temperature_Threshold(float _Temperature) {

				// Verify user is not trying to set value outside bounds
				if (_Temperature < -40) _Temperature = -40;
				if (_Temperature > 125) _Temperature = 125;

				// Calculate value to load into register
				uint8_t _Temperature_Threshold = (uint8_t)(256 * (_Temperature + 40) / 165);

				// Write Register
				I2C_Functions::Write_Register(0x0A, _Temperature_Threshold, true);

			}
		
			// Set High Temperature Threshold Function
			void Set_High_Temperature_Threshold(float _Temperature) {

				// Verify user is not trying to set value outside bounds
				if (_Temperature < -40) _Temperature = -40;
				if (_Temperature > 125) _Temperature = 125;

				// Calculate value to load into register
				uint8_t _Temperature_Threshold = (uint8_t)(256 * (_Temperature + 40) / 165);

				// Write Register
				I2C_Functions::Write_Register(0x0B, _Temperature_Threshold, true);

			}

			// Set High Humidity Threshold Function
			void Set_High_Humidity_Threshold(float _Humidity) {

				// Verify user is not trying to set value outside bounds
				if (_Humidity < 0) _Humidity = 0;
				if (_Humidity > 100) _Humidity = 100;

				// Calculate value to load into register
				uint8_t _Humidity_Threshold = (uint8_t)(256 * (_Humidity) / 100);

				// Write Register
				I2C_Functions::Write_Register(0x0D, _Humidity_Threshold, true);

			}

			// Set Low Humidity Threshold Function
			void Set_Low_Humidity_Threshold(float _Humidity) {

				// Verify user is not trying to set value outside bounds
				if (_Humidity < 0) _Humidity = 0;
				if (_Humidity > 100) _Humidity = 100;

				// Calculate value to load into register
				uint8_t _Humidity_Threshold = (uint8_t)(256 * (_Humidity) / 100);

				// Write Register
				I2C_Functions::Write_Register(0x0C, _Humidity_Threshold, true);

			}

			// Read Temperature Function
			float Temperature(const uint8_t _Measurement_Count = 1) {

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
						_Measurement_Array[_Read_ID] = (this->Sensor.Calibration.Gain_T * ((float)_Measurement_Raw * 165 / 65536 - 40)) + this->Sensor.Calibration.Offset_T;

					} else {

						// Calculate Measurement
						_Measurement_Array[_Read_ID] = (float)_Measurement_Raw * 165 / 65536 - 40;

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
						_Measurement_Array[_Read_ID] = (this->Sensor.Calibration.Gain_H * (((float)_Measurement_Raw / 65536 ) * 100.0)) + this->Sensor.Calibration.Offset_H;

					} else {

						// Calculate Measurement
						_Measurement_Array[_Read_ID] = ((float)_Measurement_Raw / 65536 ) * 100.0;

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

			/* I2C Functions */

			// Read address Function
			uint8_t Address(void) {

				// Return Address
				return(I2C_Functions::Variables.Device.Address);

			}

			// Read detect Function
			bool Detect(void) {

				// Return Address
				return(I2C_Functions::Variables.Device.Detect);

			}

			// Read Mux Channel Function
			uint8_t Mux_Channel(void) {

				// Return Address
				return(I2C_Functions::Variables.Multiplexer.Channel);

			}

	};

	// MPL3115A2 Pressure Sensor Class
	class MPL3115A2 : private I2C_Functions {

		// Private Context
		private:

			// MPL3115A2 Sensor Variable Structure.
			struct MPL3115A2_Struct {

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

			}

			// MPL3115A2 Begin Function
			void Begin(void) {

				// Start I2C Communication
				I2C_Functions::Begin();

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

				// Set CTRL_REG1 Register
				I2C_Functions::Write_Register(0x26, 0x39, false);

				// Set PT_DATA_CFG Register
				I2C_Functions::Write_Register(0x13, 0x07, false);

				// Define Variables
				uint8_t Ready_Status_Try_Counter = 0;

				// Wait for Measurement Complete
				while (!I2C_Functions::Read_Register_Bit(0x00, 2)) {
					
					// Increase Counter
					Ready_Status_Try_Counter += 1;
					
					// Control for Wait Counter
					if (Ready_Status_Try_Counter > 50) return(-102);

					// Ready Status Wait Delay
					delay(1);
					
				}

				// Define Measurement Read Array
				float _Measurement_Array[_Measurement_Count];

				// Read Loop For Read Count
				for (int Read_ID = 0; Read_ID < _Measurement_Count; Read_ID++) {

					// Define Variables
					uint8_t _MPL3115A2_Data[3];

					// Read Delay
					delay(5);

					// Read Register
					I2C_Functions::Read_Multiple_Register(0x01, _MPL3115A2_Data, 3, false);

					// Define Variables
					uint32_t _Measurement_Raw = 0;

					// Combine Read Bytes
					_Measurement_Raw = _MPL3115A2_Data[0];
					_Measurement_Raw <<= 8;
					_Measurement_Raw |= _MPL3115A2_Data[1];
					_Measurement_Raw <<= 8;
					_Measurement_Raw |= _MPL3115A2_Data[2];
					_Measurement_Raw >>= 4;
					
					// Control for Calibration
					if (this->Sensor.Calibration.Enable) {

						// Calculate Measurement
						_Measurement_Array[Read_ID] = (this->Sensor.Calibration.Gain * ((_Measurement_Raw / 4.00 ) / 100)) + this->Sensor.Calibration.Offset;

					} else {

						// Calculate Measurement
						_Measurement_Array[Read_ID] = (_Measurement_Raw / 4.00 ) / 100;

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

			};

			/* I2C Functions */

			// Read address Function
			uint8_t Address(void) {

				// Return Address
				return(I2C_Functions::Variables.Device.Address);

			}

			// Read detect Function
			bool Detect(void) {

				// Return Address
				return(I2C_Functions::Variables.Device.Detect);

			}

			// Read Mux Channel Function
			uint8_t Mux_Channel(void) {

				// Return Address
				return(I2C_Functions::Variables.Multiplexer.Channel);

			}

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
					float Value_ = Data_Array.Average(_Arithmetic_Average_);

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

			// SDP810 Sensor Variable Structure.
			struct SDP810_Struct {

				// Calibration Structure.
				struct Calibration_Struct{

					// Measurement Calibration Enable Variable (if set true library make calibration).
					bool Enable 			= false;

					// Calibration (aX+B) Gain Variable
					float Gain 				= 1;

					// Calibration (aX+B) Offset Variable
					float Offset 			= 0;

				} Calibration;

			} Sensor;

			// Stop Continuous Measurement Function
			void Stop_Continuous_Measurement(void) {

				// Write Command
				I2C_Functions::Write_Register(0x3F, 0xF9, true);

			}

			// Start Continuous Measurement Function
			void Start_Continuous_Measurement_With_Diff_Pressure_TComp(void) {

				// Write Command
//				I2C_Functions::Write_Register(0x36, 0x1E, true);	// Instant
				I2C_Functions::Write_Register(0x36, 0x15, true);	// Average

				// Command Delay
				delay(20);

			}

			// Read Measurement Raw Function
			void Read_Measurement_RAW(uint16_t & _DP, uint16_t & _Temp) {

				// Define Variables
				uint8_t _SDP810_Data[9];

				// Read Register
				I2C_Functions::Read_Multiple_Register_u16_NoCMD(_SDP810_Data, 9);

				//  0	  1      2	     3      4      5      6      7      8
				// ------------------------------------------------------------
				// DP     DP     CRC    Temp   Temp   CRC    Scale  Scale  CRC
				// ------------------------------------------------------------
				// 0x00 - 0x02 - 0xE3 - 0x13 - 0x65 - 0x8C - 0x00 - 0x3C - 0x39
				// 0xFF - 0xFD - 0xCE - 0x13 - 0x66 - 0xDF - 0x00 - 0x3C - 0x39

				// Combine Read Bytes
				_DP = ((uint16_t)_SDP810_Data[0] << 8) | ((uint16_t)_SDP810_Data[1]);

				// Combine Read Bytes
				_Temp = ((uint16_t)_SDP810_Data[3] << 8) | ((uint16_t)_SDP810_Data[4]);

			}

			// Convert Temperature Raw To Celsius Function
			float Convert_Temperature_Raw_To_Celsius(int16_t _Temperature_Raw) {

				// Return Temperature
				return((float)_Temperature_Raw / 200.0);

			}

			// Read Measurement Function
			void Read_Measurement(float & _DP, float & _Temp) {

				// Define Variables
				uint16_t _DP_Raw, _Temp_Raw;

				// Read Measurement Raw
				this->Read_Measurement_RAW(_DP_Raw, _Temp_Raw);

				// Convert to Signed Value
				int _DP_Raw_Signed = static_cast<int>(_DP_Raw);

				// Calculate Measurement
				_DP = (float)_DP_Raw_Signed / 60;

				// Calculate Measurement
				_Temp = this->Convert_Temperature_Raw_To_Celsius(_Temp_Raw);

			}

		public:

			// Declare Variables
			char Product_Number[11];
			char Serial_Number[19];

			// Construct a new SDP810 object
			SDP810(bool _Multiplexer_Enable, uint8_t _Multiplexer_Channel) : I2C_Functions(__I2C_Addr_SDP810__, _Multiplexer_Enable, _Multiplexer_Channel) {

			}

			// Begin Sensor
			void Begin(void) {

				// Start I2C Communication
				I2C_Functions::Begin();

				// Stop Continuous Measurement
				this->Stop_Continuous_Measurement();

				// Read Product Identification
				this->Read_Product_Identifier();

				// Start Continuous Measurement
				this->Start_Continuous_Measurement_With_Diff_Pressure_TComp();

			}

			// Read Product Identification Function
			void Read_Product_Identifier(void) {

				// Write Command
				I2C_Functions::Write_Register(0x36, 0x7C, true);

				// Declare Variables
				uint8_t _SDP810_Data[18];

				// Read Register
				I2C_Functions::Read_Multiple_Register_u16(0xE102, _SDP810_Data, 18, true);

				//  0      1      2	     3      4      5      6      7      8      9      10     11     12     13    14     15     16     17
				// 0x03 - 0x02 - 0xCE - 0x0A - 0x01 - 0x5E - 0x00 - 0x00 - 0x81 - 0x00 - 0x00 - 0x81 - 0x89 - 0xD6 - 0xC3 - 0x57 - 0x7B - 0xDA

				// Clear Arrays
				memset(this->Product_Number, '\0', 11);

				// Convert the 64-bit serial number to a hex string
				sprintf(this->Product_Number, "0x%02X%02X%02X%02X", _SDP810_Data[0], _SDP810_Data[1], _SDP810_Data[3], _SDP810_Data[4]);

				// Clear Arrays
				memset(this->Serial_Number, '\0', 19);

				// Convert the 64-bit serial number to a hex string
				sprintf(this->Serial_Number, "0x%02X%02X%02X%02X%02X%02X%02X%02X", _SDP810_Data[6], _SDP810_Data[7], _SDP810_Data[9], _SDP810_Data[10], _SDP810_Data[12], _SDP810_Data[13], _SDP810_Data[15], _SDP810_Data[16]);

			}

			// Read Pressure Function
			float Pressure(const uint8_t _Measurement_Count = 1) {

				// Define Measurement Read Array
				float _Pressure_Measurement_Array[_Measurement_Count];
				float _Temperature_Measurement_Array[_Measurement_Count];

				// Read Loop For Read Count
				for (uint8_t _Read_ID = 0; _Read_ID < _Measurement_Count; _Read_ID++) this->Read_Measurement(_Pressure_Measurement_Array[_Read_ID], _Temperature_Measurement_Array[_Read_ID]);

				// Control for Read Count
				if (_Measurement_Count > 1) {
					
					// Construct Object
					Array_Stats<float> Data_Array(_Pressure_Measurement_Array, _Measurement_Count);

					// Declare Variables
					float _Value;

					// Calculate Average
					if (_Measurement_Count < 5)	_Value = Data_Array.Average(_Arithmetic_Average_);
					if (_Measurement_Count >= 5) _Value = Data_Array.Average(_Sigma_Average_);

					// End Function
					return(_Value);

				} else {

					// End Function
					return(_Pressure_Measurement_Array[0]);

				} 

			}

			// Read Temperature Function
			float Temperature(void) {

				// Define Variables
				float _DP, _Temp;

				// Read Measurement
				this->Read_Measurement(_DP, _Temp);

				// End Function
				return(_Temp);

			}

			/* I2C Functions */

			// Read address Function
			uint8_t Address(void) {

				// Return Address
				return(I2C_Functions::Variables.Device.Address);

			}

			// Read detect Function
			bool Detect(void) {

				// Return Address
				return(I2C_Functions::Variables.Device.Detect);

			}

			// Read Mux Channel Function
			uint8_t Mux_Channel(void) {

				// Return Address
				return(I2C_Functions::Variables.Multiplexer.Channel);

			}

	};

	// Si1145-A19 Light Sensor Class
	class SI1145 : private I2C_Functions {

		// Private Context
		private:

			// Reset Function
			void Reset(void) {

				// Set MEAS_RATE0 Register
				I2C_Functions::Write_Register(0X08, 0x00, true);

				// Set MEAS_RATE1 Register
				I2C_Functions::Write_Register(0X09, 0x00, true);

				// Set IRQ_ENABLE Register
				I2C_Functions::Write_Register(0X04, 0x00, true);

				// Set IRQ_MODE1 Register
				I2C_Functions::Write_Register(0X05, 0x00, true);

				// Set IRQ_MODE2 Register
				I2C_Functions::Write_Register(0X06, 0x00, true);

				// Set INT_CFG Register
				I2C_Functions::Write_Register(0X03, 0x00, true);

				// Set IRQ_STATUS Register
				I2C_Functions::Write_Register(0X21, 0xFF, true);

				// Set COMMAND Register
				I2C_Functions::Write_Register(0X18, 0x01, true);

				// Command Delay
				delay(10);

				// Set HW_KEY Register
				I2C_Functions::Write_Register(0X07, 0x17, true);

				// Command Delay
				delay(10);

			}

			// Read Parameter Data Function
			uint8_t Read_Param_Data(uint8_t _Reg) {

				// Write Command
				I2C_Functions::Write_Register(0X18, _Reg | 0X80, true);

				// Read Byte
				return(I2C_Functions::Read_Register(0X2E));

			}

			// Write Parameter Data Function
			uint8_t Write_Param_Data(uint8_t _Reg, uint8_t _Value) {

				// Write Value
				I2C_Functions::Write_Register(0X17, _Value, true);

				// Write Command
				I2C_Functions::Write_Register(0X18, _Reg | 0XA0, true);

				// Read Byte
				return(I2C_Functions::Read_Register(0X2E));

			}

			// Initialize Function
			void Init(void) {

				// Enable UV reading
				I2C_Functions::Write_Register(0X13, 0X29, true);
				I2C_Functions::Write_Register(0X14, 0X89, true);
				I2C_Functions::Write_Register(0X15, 0X02, true);
				I2C_Functions::Write_Register(0X16, 0X00, true);
				this->Write_Param_Data(0x01, 0x80 | 0x20 | 0x10 | 0x01);

				// Set LED1 CURRENT(22.4mA)(It is a normal value for many LED)
				this->Write_Param_Data(0x07, 0x03);
				I2C_Functions::Write_Register(0X0F, 0X03, true);
				this->Write_Param_Data(0x02, 0x01);

				// Set ADC ENABLE
				this->Write_Param_Data(0x0B, 0x00);
				this->Write_Param_Data(0x0A, 0x07);
				this->Write_Param_Data(0x0C, 0x20 | 0x04);

				// Visible Light ADC Setting
				this->Write_Param_Data(0x11, 0x00);
				this->Write_Param_Data(0x10, 0x07);
				this->Write_Param_Data(0x12, 0x20);

				// IR ADC Setting
				this->Write_Param_Data(0x1E, 0x00);
				this->Write_Param_Data(0x1D, 0x07);
				this->Write_Param_Data(0x1F, 0x20);

				// Set interrupt enable
				I2C_Functions::Write_Register(0X03, 0X01, true);
				I2C_Functions::Write_Register(0X04, 0X01, true);

				// Auto Run Enable
				I2C_Functions::Write_Register(0X08, 0XFF, true);
				I2C_Functions::Write_Register(0X18, 0X0F, true);

			}

		// Public Context
		public:

			// Construct a new SI1145 object
			SI1145(bool _Multiplexer_Enable, uint8_t _Multiplexer_Channel) : I2C_Functions(__I2C_Addr_SI1145__, _Multiplexer_Enable, _Multiplexer_Channel) {

			}

			// Begin Sensor
			void Begin(void) {

				// Start I2C Communication
				I2C_Functions::Begin();

				// Reset Sensor
				this->Reset();

				// Initialize Sensor
				this->Init();

			}

			// Read Visible Light Function
			float Read_Visible(void) {

				// Read Register
				return((float)I2C_Functions::Read_Register_Word(0X22));

			}

			// Read IR Light Function
			float Read_IR(void) {

				// Read Register
				return((float)I2C_Functions::Read_Register_Word(0X24));

			}

			// Read UV Function
			float Read_UV(void) {

				// Read Register
				return((float)I2C_Functions::Read_Register_Word(0X2C) / 100.0);

			}

			/* I2C Functions */

			// Read address Function
			uint8_t Address(void) {

				// Return Address
				return(I2C_Functions::Variables.Device.Address);

			}

			// Read detect Function
			bool Detect(void) {

				// Return Address
				return(I2C_Functions::Variables.Device.Detect);

			}

			// Read Mux Channel Function
			uint8_t Mux_Channel(void) {

				// Return Address
				return(I2C_Functions::Variables.Multiplexer.Channel);

			}

	};

	// Weather Sensor Class
	class B108AA_Environment : private HDC2010, private MPL3115A2, private SI1145 {

		// Public Context		
		public:

			// Construct a new B108AA_Environment object
			B108AA_Environment(void) : HDC2010(true, __B108AA_MUX_HDC2010__), MPL3115A2(true, __B108AA_MUX_MPL3115__), SI1145(true, __B108AA_MUX_SI1145__) {

			}

			// Begin Sensor
			void Start(void) {

				// Start HDC2010
				HDC2010::Begin();

				// Calibrate HDC2010
				HDC2010::Set_Calibration_Parameters(1, 1.0053, -0.4102);
				HDC2010::Set_Calibration_Parameters(2, 0.9821, -0.3217);

				// Start MPL3115A2
				MPL3115A2::Begin();

				// Calibrate MPL3115A2
				MPL3115A2::Set_Calibration_Parameters(1, 0);

				// Start SI1145
				SI1145::Begin();

			}

			// Read Temperature Function
			float Temperature(void) {

				// Return Temperature
				return(HDC2010::Temperature(10));

			}

			// Read Humidity Function
			float Humidity(void) {

				// Return Humidity
				return(HDC2010::Humidity(10));

			}

			// Read Pressure Function
			float Pressure(void) {

				// Return Pressure
				return(MPL3115A2::Pressure(1));

			}

			// Read Visible Light Function
			float Visible(void) {

				// Return Visible Light
				return(SI1145::Read_Visible());

			}

			// Read IR Light Function
			float IR(void) {

				// Return IR Light
				return(SI1145::Read_IR());

			}

			// Read UV Function
			float UV(void) {

				// Return UV
				return(SI1145::Read_UV());

			}

			// Read Delta Pressure Function
			void Wind(float & _WD, float & _WS) {

				// Define Sensor Object
				SDP810 Sensor_Axis_1(true, __B108AA_MUX_SDP810_X__);
				SDP810 Sensor_Axis_2(true, __B108AA_MUX_SDP810_Y__);
				SDP810 Sensor_Axis_3(true, __B108AA_MUX_SDP810_Z__);

				// Begin Sensor
				Sensor_Axis_1.Begin();
				Sensor_Axis_2.Begin();
				Sensor_Axis_3.Begin();

				// Declare Variables
				struct Wind_Speed_Struct {

					// Axis 1 Struct
					struct Wind_Speed_A_Struct {
						float Speed;
						float X;
						float Y;
					} Axis_1;
					
					// Axis 2 Struct
					struct Wind_Speed_B_Struct {
						float Speed;
						float X;
						float Y;
					} Axis_2;

					// Axis 3 Struct
					struct Wind_Speed_C_Struct {
						float Speed;
						float X;
						float Y;
					} Axis_3;

					// X Vector Sum
					float X_Vectors_SUM;

					// Y Vector Sum
					float Y_Vector_SUM;

				} _Wind_Speed;

				// Declare Constants
				const double _Rho = 1.225;

				// Calculate Wind Speed
				_Wind_Speed.Axis_1.Speed = sqrt(2 * Sensor_Axis_1.Pressure(50) / _Rho);
				_Wind_Speed.Axis_2.Speed = sqrt(2 * Sensor_Axis_2.Pressure(50) / _Rho);
				_Wind_Speed.Axis_3.Speed = sqrt(2 * Sensor_Axis_3.Pressure(50) / _Rho);

				// Calculate Wind Direction Axis 1
				if (_Wind_Speed.Axis_1.Speed > 0) {

					// Calculate Wind Direction X Vector
					_Wind_Speed.Axis_1.X = _Wind_Speed.Axis_1.Speed * cos(radians(0));

					// Calculate Wind Direction Y Vector
					_Wind_Speed.Axis_1.Y = _Wind_Speed.Axis_1.Speed * sin(radians(0));

				} else if (_Wind_Speed.Axis_1.Speed < 0) {

					// Calculate Wind Direction X Vector
					_Wind_Speed.Axis_1.X = abs(_Wind_Speed.Axis_1.Speed) * cos(radians(180));

					// Calculate Wind Direction Y Vector
					_Wind_Speed.Axis_1.Y = abs(_Wind_Speed.Axis_1.Speed) * sin(radians(180));

				} else if (_Wind_Speed.Axis_1.Speed == 0) {

					// Calculate Wind Direction X Vector
					_Wind_Speed.Axis_1.X = 0;

					// Calculate Wind Direction Y Vector
					_Wind_Speed.Axis_1.Y = 0;

				}

				// Calculate Wind Direction Axis 2
				if (_Wind_Speed.Axis_2.Speed > 0) {

					// Calculate Wind Direction X Vector
					_Wind_Speed.Axis_2.X = _Wind_Speed.Axis_2.Speed * cos(radians(60));

					// Calculate Wind Direction Y Vector
					_Wind_Speed.Axis_2.Y = _Wind_Speed.Axis_2.Speed * sin(radians(60));

				} else if (_Wind_Speed.Axis_2.Speed < 0) {

					// Calculate Wind Direction X Vector
					_Wind_Speed.Axis_2.X = abs(_Wind_Speed.Axis_2.Speed) * cos(radians(240));

					// Calculate Wind Direction Y Vector
					_Wind_Speed.Axis_2.Y = abs(_Wind_Speed.Axis_2.Speed) * sin(radians(240));

				} else if (_Wind_Speed.Axis_2.Speed == 0) {

					// Calculate Wind Direction X Vector
					_Wind_Speed.Axis_2.X = 0;

					// Calculate Wind Direction Y Vector
					_Wind_Speed.Axis_2.Y = 0;

				}

				// Calculate Wind Direction Axis 3
				if (_Wind_Speed.Axis_3.Speed > 0) {

					// Calculate Wind Direction X Vector
					_Wind_Speed.Axis_3.X = _Wind_Speed.Axis_3.Speed * cos(radians(120));

					// Calculate Wind Direction Y Vector
					_Wind_Speed.Axis_3.Y = _Wind_Speed.Axis_3.Speed * sin(radians(120));

				} else if (_Wind_Speed.Axis_3.Speed < 0) {

					// Calculate Wind Direction X Vector
					_Wind_Speed.Axis_3.X = abs(_Wind_Speed.Axis_3.Speed) * cos(radians(300));

					// Calculate Wind Direction Y Vector
					_Wind_Speed.Axis_3.Y = abs(_Wind_Speed.Axis_3.Speed) * sin(radians(300));

				} else if (_Wind_Speed.Axis_3.Speed == 0) {

					// Calculate Wind Direction X Vector
					_Wind_Speed.Axis_3.X = 0;

					// Calculate Wind Direction Y Vector
					_Wind_Speed.Axis_3.Y = 0;

				}

				// Calculate X Vectors Sum
				_Wind_Speed.X_Vectors_SUM = _Wind_Speed.Axis_1.X + _Wind_Speed.Axis_2.X + _Wind_Speed.Axis_3.X;

				// Calculate Y Vectors Sum
				_Wind_Speed.Y_Vector_SUM = _Wind_Speed.Axis_1.Y + _Wind_Speed.Axis_2.Y + _Wind_Speed.Axis_3.Y;

				// Calculate Wind Total Speed
				_WS = sqrt(pow(_Wind_Speed.X_Vectors_SUM, 2) + pow(_Wind_Speed.Y_Vector_SUM, 2));

				// Calculate Wind Direction
				_WD = degrees(atan2(_Wind_Speed.Y_Vector_SUM, _Wind_Speed.X_Vectors_SUM));

				// Control for Wind Direction
				if (_WD < 0) _WD += 360;

			}

	};

#endif /* defined(__Environment__) */
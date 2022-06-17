#ifndef __Environment__
#define __Environment__

// Define Arduino Library
#ifndef __Arduino__
#include <Arduino.h>
#endif

// HDC2010 Class
class HDC2010 : public I2C_Functions {

	private:

		// Sensor Variables
		struct HDC2010_Struct {
			uint8_t TWI_Address = 0x40;
			bool Mux_Enable = false;
			uint8_t Mux_Channel = 0;
			uint8_t Read_Count = 10;
			bool Calibration = false;
		} Sensor;

	public:

		// Constructor
		HDC2010(bool _Multiplexer_Enable, uint8_t _Multiplexer_Channel, uint8_t _Measurement_Count = 1, bool _Calibration_Enable = false) : I2C_Functions(__I2C_Addr_HDC2010__, _Multiplexer_Enable, _Multiplexer_Channel) {

			// Set Measurement Count
			this->Sensor.Read_Count = _Measurement_Count;

			// Enable Calibration
			this->Sensor.Calibration = _Calibration_Enable;

			// Set Multiplexer Variables
			this->Sensor.Mux_Enable = _Multiplexer_Enable;
			this->Sensor.Mux_Channel = _Multiplexer_Channel;

		}

		// Temperature
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
			if (this->Sensor.Calibration) Value_ = (1.0053 * Value_) -0.4102;

			// Control For Sensor Range
			if (Value_ < -40 or Value_ > 125) Value_ = -101;

			// End Function
			return(Value_);
			
		}

		// Humidity
		float Humidity(void) {

			// Reset Sensor
			Set_Register_Bit(0x0E, 7, false);

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
			if (this->Sensor.Calibration) Value_ = (0.9821 * Value_) -0.3217;

			// Control For Sensor Range
			if (Value_ < 0 or Value_ > 100) Value_ = -101;

			// End Function
			return(Value_);
			
		}

};

// Analog Read Class
class Analog {

	private:

		// Analog Class Struct Definition
		struct Analog_Struct {

			// Read Count
			uint8_t Read_Count;

			// Calibration Parameters
			bool Calibration;

			// Calibration Parameters
			float Cal_a;
			float Cal_b;

		} Measurement;

	public:

		// Statistical Parameeters
		float Standart_Deviation;

		// Constructor
		Analog(uint8_t _Channel, uint8_t _Read_Count, bool _Calibration, float _Cal_a, float _Cal_b) {

			// Set Channel Variable
			_Channel &= 0b00000111;

			// Set Variables
			this->Measurement.Read_Count = _Read_Count;
			this->Measurement.Calibration = _Calibration;
			this->Measurement.Cal_a = _Cal_a;
			this->Measurement.Cal_b = _Cal_b;

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

		// Read
		double Read(void) {

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
					_Array[Read_ID] = (this->Measurement.Cal_a * _Pressure) + this->Measurement.Cal_b;

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
			this->Standart_Deviation = Data_Array.Standard_Deviation();

			// End Function
			return(_Data);
			
		}

};

#endif /* defined(__Environment__) */

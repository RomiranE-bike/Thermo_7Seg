/*
 * thermocuple.h
 *
 * Created: 2/22/2024 12:48:14
 *  Author: me
 */ 

//-------------------------------------------------------------------------------------------------------------
#ifndef THERMOCUPLE_H_
#define THERMOCUPLE_H_
//-------------------------------------------------------------------------------------------------------------
//convert to Celsius degree//
// read the temperature input convert to Celsius degree

// actual reference voltage of the board, in mV (as measured)
#define     VREF_MV   4930     // 5V   when analog Reference is DEFAULT
//#define   VREF_MV   1060     // 1.1V when analog Reference is INTERNAL

#define    TYPE_K_SENSOR_PIN   PC0
#define    LM35_SENSOR_PIN     PC1

#define        R1                        150                  // non-inverting amplifier R1 in ohm
#define        R2                        15000                // non-inverting amplifier R2 in ohm
#define        GAIN                      (1 + R2/R1)          // non-inverting amplifier gain
#define        MV_PER_ANALOG_DIVISION    (VREF_MV / 1023.0)
#define        CELCIUS_PER_MV            (1.0 / 0.041)        // 41µV per °C linear approximation for Type K > °C
const float    type_k_ReadingScaler     = CELCIUS_PER_MV / GAIN * MV_PER_ANALOG_DIVISION;
//-------------------------------------------------------------------------------------------------------------
int           analog_Pin = 0;
int           sensor_value = 0;// variable to store the ADC value from PC0
float         temperature;// Temperature value in Celsius degree.
float         fahrenheit;// Temperature value in  Fahrenheit degree.
float         gain = 0.00488;
float         ref  = 1.25313;

double sensor_value = (double(sensor_read(analog_Pin)) * gain - ref)/0.005;
//-------------------------------------------------------------------------------------------------------------
void volt_celsius(void){

	sensor_value = sensor_read(analog_Pin);                   // read the input pin
	temperature = (float(sensor_value) * gain - ref)/0.005; // convert to Celsius degree
	fahrenheit = temperature * 1.8 + 32.0;		// convert to Fahrenheit degree
	
}

//-------------------------------------------------------------------------------------------------------------
float sampleTemperatureOffset(){
	float   analog_reading = 1.0 * sensor_read(analog_Pin);
	return  analog_reading * type_k_ReadingScaler ;
}

//-------------------------------------------------------------------------------------------------------------

#endif /* THERMOCUPLE_H_ */
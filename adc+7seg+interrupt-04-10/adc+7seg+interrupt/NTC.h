/*
 * NTC.h
 *
 * Created: 2/22/2024 12:43:56
 *  Author: me
 */ 

//-------------------------------------------------------------------------------------------------------------
#ifndef NTC_H_
#define NTC_H_
//-------------------------------------------------------------------------------------------------------------
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
//-------------------------------------------------------------------------------------------------------------
double Thermister;//NTC
double Kelvin;
double Celsius;
double Fahrenheit;

#define ntc_pin  PC0// Pin,to which the voltage divider is connected
#define vd_power_pin  2// 5V for the voltage divide
#define nominal_resistance 100000//Nominal resistance 100k at 25?C
#define nominal_temeprature  25// temperature for nominal resistance (almost always 25? C)
#define samplingrate  5// Number of samples
#define beta  3950// The beta coefficient or the B value of the thermistor (usually 3000-4000) 
//check the data sheet for the accurate value
#define Rref 100000//Value of  fix series resistor used for the voltage divider 100k

const double  V_ref = 4.6099925;// AVcc
const double  V_in = 3.3;// AMS1117 3.3v regulator
const double  Rfix = 100000;// 100k ohm series resistor
const double  Beta = 3950;
const double  Adc_Resolution = 1023;// 10-bit adc

const double para_A = 0.001129148;// thermistor equation parameters
const double para_B = 0.000234125;
const double para_C = 0.0000000876741;

double V_out, Rth, Temperature, adc_value, ohm;
//-------------------------------------------------------------------------------------------------------------
double NTC_function_1(uint16_t sens_value);
double NTC_function_2(uint16_t sens_value);
//-------------------------------------------------------------------------------------------------------------
double NTC_function_1(uint16_t sens_value){
		//V_out = V_in*(Rth/(Rfix+Rth));
		//Rth = (V_out*Rfix) / (V_in-V_out);
		V_out =  (sens_value * V_ref ) / Adc_Resolution;// calculate voltage
		Rth =  (V_out * Rfix) / (V_in - V_out);
		Kelvin = 1.0/(((log(Rth / Rfix)) / Beta) + (1.0 / (nominal_temeprature + 273.15)));
		Celsius = Kelvin - 273.15;
		Fahrenheit  = Celsius * 1.8 + 32;
		/*
		display_char(null,u,c,o);//display_char(uint16_t ch3,uint16_t ch2,uint16_t ch1,uint16_t ch0)
		_delay_ms(2000);
		display_num(V_out,250);//display_num(uint16_t number,uint16_t cycle)
		
		display_char(null,t,h,e);//display_char(uint16_t ch3,uint16_t ch2,uint16_t ch1,uint16_t ch0)
		_delay_ms(2000);
		display_num(Rth,250);//display_num(uint16_t number,uint16_t cycle)
		*/
        return Celsius;
}
//------------------------------------------------------------------------------------------------------------- 
double NTC_function_2(uint16_t sens_value){
	//V_out = V_in*(Rth/(Rfix+Rth));
	//Rth = (V_out*Rfix) / (V_in-V_out);
	V_out = (sens_value * V_ref) / Adc_Resolution;//V_out = V_in * (R_out/(R_in + R_out))
	//Rth = ((V_out * Rfix )/ V_in) - Rfix;//R_out = ((V_out*R_in)/V_in)-R_in
	Rth =  (V_out * Rfix) / (V_in - V_out);
	//  Stein hart-Hart Thermistor Equation:
	//  Temperature in Kelvin = 1 / (A + B[log_neper(R)] + C[log_neper(R)]^3)
	//  where A = 0.001129148, B = 0.000234125 and C = 8.76741*10^-8
	//  Temperature in kelvin
	Temperature = (1.0 / (para_A + (para_B * log(Rth))+ (para_C * pow((log(Rth)), 3))));
	Celsius = Temperature - 273.15;// Temperature in degree Celsius		
	
	return Celsius;
}
//-------------------------------------------------------------------------------------------------------------
/*
Vout = Vin*[R2/(R1+R2)]
R2=(Vout*R1) / (Vin-Vout)
//-------------------------------------------------------------------------------------------------------------
int ThermistorPin = A1;
float voltageDividerR1 = 10000;
float BValue = 3470;
float R1 = 5000;
float T1 = 298.15;
float R2 ;
float T2 ;

float a ;
float b ;
float c ;
float d ;
float e = 2.718281828 ;
if(millis() &gt;= tempLastSample + 1)
{
	tempSampleRead = analogRead(ThermistorPin);
	tempSampleSum = tempSampleSum+tempSampleRead;
	tempSampleCount = tempSampleCount+1;
	tempLastSample = millis();
}

if(tempSampleCount == 1000)
{
	tempMean = tempSampleSum / tempSampleCount;
	R2 = (voltageDividerR1*tempMean)/(1023-tempMean);
	
	a = 1/T1;
	b = log10(R1/R2);
	c = b / log10(e);
	d = c / BValue ;
	T2 = 1 / (a- d);
	Serial.print(T2 - 273.15,decimalPrecision);
	Serial.println(" °C");
	
	tempSampleSum = 0;
	tempSampleCount = 0;
}
//-------------------------------------------------------------------------------------------------------------
float Vin=5.0;     // [V]
float Rt=10000;    // Resistor t [ohm]
float R0=10000;    // value of rct in T0 [ohm]
float T0=298.15;   // use T0 in Kelvin [K]
float Vout=0.0;    // Vout in A0
float Rout=0.0;    // Rout in A0
// use the data sheet to get this data.
float T1=273.15;      // [K] in datasheet 0º C
float T2=373.15;      // [K] in datasheet 100° C
float RT1=35563;   // [ohms]  resistence in T1
float RT2=549;    // [ohms]   resistence in T2
float beta=0.0;    // initial parameters [K]
float Rinf=0.0;    // initial parameters [ohm]
float TempK=0.0;   // variable output
float TempC=0.0;   // variable output
//parameters
beta=(log(RT1/RT2))/((1/T1)-(1/T2));
Rinf=R0*exp(-beta/T0);
Vout=Vin*((float)(analogRead(0))/1024.0); // calc for ntc
Rout=(Rt*Vout/(Vin-Vout));
TempK=(beta/log(Rout/Rinf)); // calc for temperature
TempC=TempK-273.1
//-------------------------------------------------------------------------------------------------------------
adc_value = sens_value;
if (adc_value < 1){
	// never divide by zero:
	adc_value=1;
	ohm = R2 / (( 1024.0 / adc_value ) - 1 );
}
//-------------------------------------------------------------------------------------------------------------
// 10bit adc=0..1023 over a voltage range from 0-5V
// U_ref=5V = AVcc
// 10000 / ( 1024/adc_value  - 1) -> ohm

// +3.3V V_in
// ++++
//    /
//    \
//    /   R_in  Rfix
//    \
//    |====>> V_out
//    /
//    \
//    /   R_out  Rth
//    \
// ----
//V_out = V_in * (R_out/(R_in + R_out))
//R_out = ((V_out*R_in)/V_in)-R_in
//-------------------------------------------------------------------------------------------------------------
// +3.3V V_in
// ++++
//    /
//    \
//    /   R_in   Rth
//    \
//    |====>> V_out
//    /
//    \
//    /   R_out   Rfix
//    \
// ----
//V_out = V_in * (R_out/(R_in + R_out))
//R_in = ((V_out/V_in)/R_out) - R_out
*/
//-------------------------------------------------------------------------------------------------------------
/*
// 10bit adc=0..1023 over a voltage range from 0-5V
// U_ref=5V = AVcc
// 10000 / ( 1024/adc_value  - 1) -> ohm
//
// +5V
// ++++
//    /
//    \
//    /
//    \  100k constant resistor  R1
//    |====>> U_adc on  ADC pin
//    /
//    \
//    /
//    \  R_ntc 100k              R2
// ----
//
// adc_value = U_adc * (1024/U_ref)
// R_ntc = 100K * ( 1 / ((5V/U_adc) -1) )
*/
//-------------------------------------------------------------------------------------------------------------
//Vout = (1 + R2/R1) * Vin
//-------------------------------------------------------------------------------------------------------------
#endif /* NTC_H_ */
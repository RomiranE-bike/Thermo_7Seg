/*
 * adc.h
 * ver.2.0
 * Created: 1/30/2024 17:03:29
 *  edited: me
 */ 

//-------------------------------------------------------------------------------------------------------------
#ifndef ADC_H_
#define ADC_H_
//-------------------------------------------------------------------------------------------------------------
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
//-------------------------------------------------------------------------------------------------------------
uint16_t extra_button_value;
uint16_t sensor_value;
uint16_t adc_read(uint16_t channel);
uint16_t sensor_read(uint16_t sensor);
uint16_t read_extera_button(uint16_t channel);
//-------------------------------------------------------------------------------------------------------------
// ADC sample at specified channel, return 10-bit result
uint16_t adc_read(uint16_t channel){
	
	    ADMUX |= (channel & 0b0000111);//set ADC channel : channel must be 0 to 7  (ADC0....ADC7)
		//ADMUX |= (1<<REFS1) | (1<<REFS0);//Internal 2.56V Voltage Reference with external capacitor at AREF pin
		ADMUX |= (1<<REFS0);//AVCC reference voltage  with external capacitor at AREF pin
		
		ADCSRA |=(0 << ADPS2)|(1 << ADPS1)|(1 << ADPS0);// prescalar: 8 => 1MHz/8 = 128kHz
		//ADCSRA |=(1 << ADPS2)|(1 << ADPS1)|(1 << ADPS0);// prescalar: 128 => 1MHz/128 = 8kHz
		
		ADCSRA|= (1<<ADEN);//ADC Conversion enable
		//ADCSRA|= (1<<ADFR);//ADC Free Running Select
		//ADCSRA|= (1<<ADIE);//ADC Interrupt Enable
		ADCSRA|= (1<<ADSC);//ADC Start Conversion
		while(!(ADCSRA & (1<<ADIF)));// waiting for ADIF, conversion complete,Clear flag
				ADCSRA|=(1<<ADIF); // clearing of ADIF, it is done by writing 1 to it
		return (ADC);// return results	
}
//-------------------------------------------------------------------------------------------------------------
uint16_t sensor_read(uint16_t sensor){

		unsigned char sample;
		sensor_value = adc_read(sensor);//analog to digital conversion
			for(sample=0;sample<16;sample++){
				sensor_value += adc_read(sensor);// read ADC samples
					  }
		sensor_value >>= 4;// take average of the 16 samples 'adc_val /= 16'  0b 1111 1111 >> 4 = 0b 0000 1111
		return  sensor_value;	   
}
//-------------------------------------------------------------------------------------------------------------
/*
uint16_t read_extera_button(uint16_t channel){

		unsigned char sample;
		extra_button_value = adc_read(channel);//analog to digital conversion
		for(sample=0;sample<16;sample++){
			extra_button_value += adc_read(channel);// read ADC samples
		}
		extra_button_value >>= 4;  // take average of the 16 samples 'adc_val /= 16'  0b 1111 1111 >> 4 = 0b 0000 1111
		extra_button_value = (extra_button_value *5 + 512)/1024;
		return  extra_button_value;
		
		
		temp=temp/8; // >>3
		temp_c=(float)(temp)*.25;
		temp_f=temp_c*(9/5)+32;
	
}
*/
//-------------------------------------------------------------------------------------------------------------
#endif /* ADC_H_ */
//-------------------------------------------------------------------------------------------------------------
//ADC Control and Status//page 205
/*
ADC Control and Status
Register A – ADCSRA
Bit 7 6 5 4 3 2 1 0
ADEN ADSC ADFR ADIF ADIE ADPS2 ADPS1 ADPS0 ADCSRA
Read/Write R/W R/W R/W R/W R/W R/W R/W R/W
Initial Value 0 0 0 0 0 0 0 0

Bit 7 – ADEN: ADC Enable
Bit 6 – ADSC: ADC Start Conversion
Bit 5 – ADFR: ADC Free Running Select
Bit 4 – ADIF: ADC Interrupt Flag
Bit 3 – ADIE: ADC Interrupt Enable
Bits 2:0 – ADPS2:0: ADC Prescaler Select Bits


Table 76. ADC Prescaler Selections
ADPS2 ADPS1 ADPS0 Division Factor
0 0 0 2
0 0 1 2
0 1 0 4
0 1 1 8
1 0 0 16
1 0 1 32
1 1 0 64
1 1 1 128

The ADC Data Register – ADCL and ADCH

ADLAR = 0
Bit 15 14 13 12 11 10 9 8
– – – – – – ADC9 ADC8 ADCH
ADC7 ADC6 ADC5 ADC4 ADC3 ADC2 ADC1 ADC0 ADCL
7 6 5 4 3 2 1 0
Read/Write R R R R R R R R
R R R R R R R R
Initial Value 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0
Bit 15 14 13 12 11 10 9 8

ADLAR = 1
ADC9 ADC8 ADC7 ADC6 ADC5 ADC4 ADC3 ADC2 ADCH
ADC1 ADC0 – – – – – – ADCL
7 6 5 4 3 2 1 0
Read/Write R R R R R R R R
R R R R R R R R
Initial Value 0 0 0 0 0 0 0 0
0 0 0 0 0 0 0 0

ADC9:0: ADC Conversion result


//-----------------------------------------------------------------------------------------------------
		//ADMUX &= ~(1 << ADLAR);// Right adjust ADC result for 10 bit reading (default)
		// -    -    -    -    -    -   ADC9 ADC8    ADCH
		//ADC7 ADC6 ADC5 ADC4 ADC3 ADC2 ADC1 ADC0    ADCL
		//ADMUX |=  (1 << ADLAR);// Left adjust ADC result to allow easy 8 bit reading
		//ADC9 ADC8 ADC7 ADC6 ADC5 ADC4 ADC3 ADC2   ADCH
		//ADC1 ADC0  -    -    -    -    -    -     ADCL
		----------------------------------------------------------------------------------		
*/
//-------------------------------------------------------------------------------------------------------------
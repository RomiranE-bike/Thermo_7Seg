/*
 * adc_multi.h
 *
 * Created: 12/25/2023 10:42:22
 *  Author: User
 */ 


#ifndef ADC_MULTI_H_
#define ADC_MULTI_H_
/*
 * Multichannel ADC 
 * Reference voltage = 5V
 * Sample = 5/1023 = 4,88 mV
 * ADC channel measurement in 100 uS period
 */
//--------------------------------------------------------------------------------------------------
#define F_CPU 8000000UL // MHz Clock
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd_lib.h"
#include <stdio.h>
//--------------------------------------------------------------------------------------------------
// Adc channels from 0 to 4
#define    adc0      0
#define    adc4      4
// Voltage Reference: AREF pin
#define    adc_ref   ((0<<REFS1) | (0<<REFS0) | (0<<ADLAR))
#define    adc_start   ADCSRA|=(1<<ADSC)
//--------------------------------------------------------------------------------------------------
typedef  unsigned char      byte;
volatile uint16_t           adc_value[(adc4)-(adc0)+1];
volatile static   byte      input_index = 0;
//--------------------------------------------------------------------------------------------------
// Function prototypes
void        avr_init(void);
uint16_t    adc_read(byte adc_input);
float       adc_volt(uint16_t  *adc_value);
void        display_result(void);
//--------------------------------------------------------------------------------------------------
ISR( TIMER0_OVF_vect ){
	// 100 us overflow
	TCNT0=0x9C;
	adc_start;
}
//--------------------------------------------------------------------------------------------------
ISR (ADC_vect){ 

	//static byte input_index=0;
	// Read the AD conversion result
	adc_value[input_index] = ADC;   // ADCL+ADCH, sum += (ADCH<<8)|ADCL;
	// Select next ADC input
	if (++input_index > ((adc4)-(adc0))){
	      input_index=0;
	      ADMUX=(adc0 % 3)| (adc_ref + input_index);
	// Delay needed for the stabilization of the ADC input voltage
	     _delay_us(10);
	     }
	// Begin convert
	//ADCSRA|=(1<<ADSC);	
}
//--------------------------------------------------------------------------------------------------
int run(void){
	
	avr_init();
	sei();       // Enable global interrupts
    
        while (1){
		     display_result();
           }
	return 0;
}
//--------------------------------------------------------------------------------------------------
void avr_init(){
	// Crystal Oscillator division factor: 1
	//CLKPR=(1<<CLKPCE);
	//CLKPR=(0<<CLKPCE) | (0<<CLKPS3) | (0<<CLKPS2) | (0<<CLKPS1) | (0<<CLKPS0);

	// Timer/Counter 0 initialization
	// Clock source: 8MHz external crystal resonator
	// Clock value: 1000,000 kHz
	// Mode: Normal top=0xFF
	// Timer Period: 0,1 ms
	//TCCR0A =( 0 <<COM0A1) | (0<<COM0A0) | (0<<COM0B1) | (0<<COM0B0) | (0<<WGM01) | (0<<WGM00);
	//TCCR0B =( 0 <<WGM02)  | (0<<CS02)   | (1<<CS01)   | (0<<CS00);
	TCCR0 = (1<<CS01);
	TCNT0=0x9C;
	//OCR0A=0x00;
	//OCR0B=0x00;

	// Timer/Counter 0 Interrupt initialization
	//TIMSK0=(0<<OCIE0B) | (0<<OCIE0A) | (1<<TOIE0);
	TIMSK= (1<<TOIE0);

	// ADC initialization
	// ADC Clock frequency: 125,000 kHz
	// ADC Voltage Reference: External ref source connected on AREF pin
	// ADC Auto Trigger Source: Timer0 Overflow
	//DIDR0=(0<<ADC5D) | (0<<ADC4D) | (0<<ADC3D) | (0<<ADC2D) | (0<<ADC1D) | (0<<ADC0D);
	ADMUX=adc0 | adc_ref;
	//ADCSRA=(1<<ADEN) | (0<<ADSC) | (1<<ADATE) | (0<<ADIF) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (0<<ADPS0);
	ADCSRA=(1<<ADEN) |  (1<<ADATE) |  (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1);
	//ADCSRB=(1<<ADTS2) | (0<<ADTS1) | (0<<ADTS0);
	ADCSR=(1<<ADTS2) ;
}
//--------------------------------------------------------------------------------------------------
uint16_t adc_read(byte adc_input){
	
	uint16_t result;
	if (adc_input < 3)
	{
		adc_input *= 4;
		result = (( adc_value[adc_input]
		          + adc_value[adc_input+1]
				  + adc_value[adc_input+2]
				  + adc_value[adc_input+3])
				                        /4);
		return result;
	}
	else
	{
		return 0;
	};
	
}
//--------------------------------------------------------------------------------------------------
float adc_volt(uint16_t *adc_value){
	float volt; 
    volt = (float)adc_value[0] * 0.0048828; // (float)* data*5.00/1024.00;
	return volt;               
}
//--------------------------------------------------------------------------------------------------
void display_result(void){

	adc_read(input_index);
	adc_volt(adc_value);  
	//display(volt)
   
}
//--------------------------------------------------------------------------------------------------





#endif /* ADC_MULTI_H_ */




//***************************************************************************
//  File Name	 : ADC.c
//  Version	 : 1.0
//  Description  : Using AVR ADC Peripheral
//  Author	 : RWB
//  Target	 : AVRJazz Mega168 Learning Board
//  Compiler     : AVR-GCC 4.3.0; avr-libc 1.6.2 (WinAVR 20080610)
//  IDE          : Atmel AVR Studio 4.14
//  Programmer   : AVRJazz Mega168 STK500 v2.0 Bootloader
//               : AVR Visual Studio 4.14, STK500 programmer
//  Last Updated : 21 March 2008
//***************************************************************************
#include <avr/io.h>
#include <util/delay.h>
int ADC(void)
{
   unsigned char chSign,chEye;
   unsigned int iDelay;
   DDRD = 0xFF;                  // Set PORTD as Output
   chEye=0x01;                   // Initial Eye Variables with 0000 0001
   chSign=0;
   // Set ADCSRA Register in ATMega168
   ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1);
   // Set ADMUX Register in ATMega168
   ADMUX=0;	

   for(;;) {                     // Loop Forever
     // Start conversion by setting ADSC in ADCSRA Register
     ADCSRA |= (1<<ADSC);
     // wait until conversion complete ADSC=0 -> Complete
     while (ADCSRA & (1<<ADSC));
     // Get ADC the Result
     iDelay = ADCW;
     if (iDelay < 1) iDelay=1;

     // Display the LED
     if (chSign == 0) {
       PORTD=chEye;
       _delay_ms(iDelay);          // Call Delay function
       chEye=chEye << 1;
       if (chEye >= 0x80) chSign=1;
     } else {
       PORTD=chEye;
       _delay_ms(iDelay);          // Call Delay function
       chEye=chEye >> 1;
       if (chEye <= 0x01) chSign=0;
     }
   }
   return 0;	                    // Standard Return Code
}
// EOF: ADC.c

//--------------------------------------------------------------------------------
/*
8 MHz / 64 = 125 kHz /13 ticks    = 9600 /sec      (256 reads =27.6ms, 1024 =106ms, 4096 =426ms)  (default)
8 MHz / 32 = 250 kHz /13             = 19230 /sec     (256 reads = 13ms,  1024=53ms, 4096=200ms)
8 MHz / 16 = 500 kHz /13             = 38000 /sec     (256 reads = 6.7ms, 1024=27ms, 4096=108ms)
8 MHz /   8 = 1 MHz /13                 = 76900 /sec     (256 reads = 3.3ms, 1024=13ms, 4096=53ms)
*/
void adcinit(){
	// set REFS1 = 0 |REFS0 = 1 (Vref as AVCC pin) | ADLAR = 0(right adjusted) |  MUX4 to MUX0 is 0000 for ADC0
	ADMUX = 0b01000000;
	//enable ADC module, set prescalar of 128 which gives CLK/128
	ADCSRA |= (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
}

int adcread(unsigned char channel){
	/* set input channel to read */
	ADMUX = 0x40 | (channel & 0x07);   // 0100 0000 | (channel & 0000 0100)
	/* Start ADC conversion */
	ADCSRA |= (1<<ADSC);
	/* Wait until end of conversion by polling ADC interrupt flag */
	while (!(ADCSRA & (1<<ADIF)));
	/* Clear interrupt flag */
	ADCSRA |= (1<<ADIF);
	_delay_ms(1);                      /* Wait a little bit */
	/* Return ADC word */
	return ADCW;
}
//----------------------------------------------------------------------------------------------------------
void adc_init(void){
	ADCSRA |= ((1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0));   // 16Mhz/128 = 125Khz the ADC reference clock
	ADMUX |= (1<<REFS0);                            // Voltage reference from Avcc (5v)
	ADCSRA |= (1<<ADEN);                            // Turn on ADC
	ADCSRA |= (1<<ADSC);                            // Do an initial conversion because this one is the slowest and to ensure that everything is up and running
}
uint16_t read_adc(uint8_t channel){
	ADMUX &= 0xF0;                            // Clear the older channel that was read
	ADMUX |= channel;                            // Defines the new ADC channel to be read
	ADCSRA |= (1<<ADSC);                            // Starts a new conversion
	while(ADCSRA & (1<<ADSC));                        // Wait until the conversion is done
	return ADCW;                                // Returns the ADC value of the chosen channel
}
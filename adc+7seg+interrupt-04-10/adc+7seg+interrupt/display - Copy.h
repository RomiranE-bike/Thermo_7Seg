/*
 * display.h
 *
 * Created: 12/24/2023 17:04:43
 *  Author: User
 */ 

//-------------------------------------------------------------------------------------------------------------
#ifndef DISPLAY_H_
#define DISPLAY_H_
/*
 * display.h
 *
 * edited: 12/21/2023 20:13
 * by : me
 * ver.01
 */ 
//-------------------------------------------------------------------------------------------------------------
/**
 *  AVR ATmega8, 
 *  segment display  
 *  LM35  sensor
 *
 */
//-------------------------------------------------------------------------------------------------------------
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
//-------------------------------------------------------------------------------------------------------------
#define common_cathode			1                // Define to 1 for Common Cathode display, 0 for Common Anode display
#define dot_enable				1                // Define to 1 to use decimal digit segment
static unsigned char    num_digits[4];              // Individual segment pattern of 3 digit number
static unsigned char    dot_pos = 3;
//volatile static         bool adc_trigger = false;
//-------------------------------------------------------------------------------------------------------------
void num_to_digits(int num, unsigned char *digits_buf);
void display_digit(int num, unsigned char dot_pos);
void disply_init(void);
//-------------------------------------------------------------------------------------------------------------
// Port and Pin definitions for seven segments
#define seg_DDR		    DDRB
#define seg_PORT	    PORTB
#define seg_A		   (1 << PB0)
#define seg_B		   (1 << PB1)
#define seg_C		   (1 << PB2)
#define seg_D		   (1 << PB3)
#define seg_E		   (1 << PB4)
#define seg_F		   (1 << PB5)
#define seg_G		   (1 << PB6)
#define seg_H		   (1 << PB7)
//-------------------------------------------------------------------------------------------------------------
// Port and pin definitions for sell 
#define sell_DDR		DDRD
#define sell_PORT	    PORTD

#define sell_1		    (1 << PD3)    /* Leftmost digit */
#define sell_2		    (1 << PD2)
#define sell_3		    (1 << PD1) 
#define sell_4		    (1 << PD0)    /* Rightmost digit */
#define Relay_1         (1 << PD4)
#define Relay_2         (1 << PD5)
#define Green_LED       (1 << PD6)
#define Buzzer          (1 << PD7)

//-----------------------------------------------------------------------------------------------
// segment definitions for digits 
enum digits_enum {
					digit_0	= (seg_A|seg_B|seg_C|seg_D|seg_E|seg_F),
					digit_1	= (seg_B|seg_C),
					digit_2 = (seg_A|seg_B|seg_G|seg_E|seg_D),
					digit_3	= (seg_A|seg_B|seg_G|seg_C|seg_D),
					digit_4 = (seg_F|seg_G|seg_B|seg_C),
					digit_5 = (seg_A|seg_F|seg_G|seg_C|seg_D),
					digit_6 = (seg_A|seg_F|seg_G|seg_C|seg_D|seg_E),
					digit_7 = (seg_A|seg_B|seg_C),
					digit_8 = (seg_A|seg_B|seg_C|seg_D|seg_E|seg_F|seg_G),
					digit_9 = (seg_A|seg_B|seg_D|seg_G|seg_F|seg_C),
					dot_pin = (seg_H),
					digit_NULL = 0,
					error_E = (seg_A|seg_D|seg_E|seg_F|seg_G)
               };
//-------------------------------------------------------------------------------------------------------------
// Array of all digits  
static unsigned char digits_arr[] = {
					digit_0, 
					digit_1, 
					digit_2, 
					digit_3, 
					digit_4, 
					digit_5, 
					digit_6, 
					digit_7, 
					digit_8, 
					digit_9,
					dot_pin,
					error_E 				
				};
//-------------------------------------------------------------------------------------------------------------
                            /*-------------------------------------------------------------------------
							    *  Converts a number from to digit pattern
								*		num : number to be converted
								*      digits_buf : array to store digit pattern for each digit of the number
								*	NOTE: Limited to 3 digit numbers, with leading zeros not displayed
								*
							--------------------------------------------------------------------------*/
//-------------------------------------------------------------------------------------------------------------	
// Initialization for AVR chip
void disply_init(void){
	
	seg_DDR = 0xFF;                                     // segment port is output
	#if common_cathode
	seg_PORT = 0x00;                                   // all segments OFF initially
	#else
	seg_PORT = 0xFF;                                   // all segments OFF initially
	#endif
	sell_DDR  |= (sell_1|sell_2|sell_3|sell_4);         //  select output pins
	#if common_cathode
	sell_PORT |= (sell_1|sell_2|sell_3|sell_4);          //  digits selected initially
	#else
	sell_PORT &= ~(sell_1|sell_2|sell_3|sell_4);         //  digits selected initially
	#endif
}
//-------------------------------------------------------------------------------------------------------------						
void num_to_digits(int num, unsigned char *digits_buf){
	
					unsigned char digit;		 
								
					digit = num/1000;					
					digits_buf[3] = digits_arr[digit];
					num -= (digit*1000);
					
					digit = num/100;	
					digits_buf[2] = digits_arr[digit];
					num -= (digit*100);
					
					digit = num/10;
					digits_buf[1] = digits_arr[digit];
					num -= (digit*10);
					
					digit = num;
					digits_buf[0] = digits_arr[digit];					
	}
//-------------------------------------------------------------------------------------------------------------
// ISR for TImer0 overflow interrupt (should come here every 2 ms)
ISR(TIMER0_OVF_vect){
	
	static unsigned char pos;
	static unsigned char counter;
	
	// Activate one digit select pin and output seven segment pattern
	#if common_cathode
	seg_PORT =   digit_NULL;  //digit_NULL
	#else
	seg_PORT = ~(digit_NULL);
	#endif
	
	if(pos == 0){
		#if common_cathode
		sell_PORT |=   (sell_2|sell_3|sell_4); sell_PORT &= ~ (sell_1);  //0b0111 0000
		#else
		sell_PORT &= ~ (sell_2|sell_3|sell_4); sell_PORT |=   (sell_1);  //0b1000 0000
		#endif
	}
	else if(pos == 1){
		#if common_cathode
		sell_PORT |=   (sell_1|sell_3|sell_4); sell_PORT &= ~ (sell_2);    //0b1011 0000
		#else
		sell_PORT &= ~ (sell_1|sell_3|sell_4); sell_PORT |=   (sell_2);    //0b0100 0000
		#endif
	}
	else if(pos == 2){
		#if common_cathode
		sell_PORT |=   (sell_1|sell_2|sell_4); sell_PORT &= ~ (sell_3);  //0b0010 0000
		#else
		sell_PORT &= ~ (sell_1|sell_2|sell_4); sell_PORT |=   (sell_3);  //0b1101 0000
		#endif
	}
	else{
		#if common_cathode
		sell_PORT |=   (sell_1|sell_2|sell_3); sell_PORT &= ~ (sell_4);  //0b1110 0000
		#else
		sell_PORT &= ~ (sell_1|sell_2|sell_3); sell_PORT |=   (sell_4);   //0b0001 0000
		#endif
	}
	
	#if common_cathode
	seg_PORT = (unsigned char)   num_digits[pos];   // Common Cathode display (HIGH bit = ON segment)	ABCDEFG
	#else
	seg_PORT = (unsigned char) ~ num_digits[pos];   // Common anode display   (LOW bit  = ON segment)   GFEDCBA
	#endif
	#if dot_enable	
	if(dot_pos == pos) {
		#if common_cathode
		seg_PORT |=   (seg_H);   // If decimal point need to be displayed
		#else
		seg_PORT &= ~ (seg_H);
		#endif
	}
	#endif
	pos++;
	if(pos > 3){pos = 0;}	                               // cycle the digit select pins
	//counter++;
	//if(counter == 15) {counter = 0;adc_trigger = true;}    // trigger for ADC at every 30 ms (15 * 2ms = 30ms)
}
//-------------------------------------------------------------------------------------------------------------
#endif /* DISPLAY_H_ */
//-------------------------------------------------------------------------------------------------------------
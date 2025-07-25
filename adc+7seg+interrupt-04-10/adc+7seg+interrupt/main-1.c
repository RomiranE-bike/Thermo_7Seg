/*
 * adc+7seg+interrupt.c
 * ver.01
 * Created: 12/24/2023 15:57:40
 * edited : me
 */ 

/*
 * ADC_LCD.c
 * ver.00
 * Created: 12/21/2023 19:17:39
 * edited : me
 */ 

//-----------------------------------------------------------------------------------------------
// File Name    : 'main.c'
// Title        : ADC single conversion within interrupt after conversion
//-----------------------------------------------------------------------------------------------
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <stdbool.h>
//#include "display.h"
//#include "adc.h"
//#include "timer_interrupt.h"
//#include "relay.h"
//-----------------------------------------------------------------------------------------------
float v_refrence   = 4.88;
float adc_value    = 0.0;
float thermo       = 0.0;
float degree       = 0.0;
float mili_volt_0  = 0.0;
float mili_volt_1  = 0.0;
int T1  =3000; //370 C
int T2  =2000; //365 C	
//-----------------------------------------------------------------------------------------------
int start_flag     = 1;
int T1_flag        = 0;
int T2_flag        = 0;
#define  sw1       PINC2
#define  sw2       PINC3
#define  up        PINC4
#define  down      PINC5

#define BUTTON_PORT PORTC           /* PORTx - register for button output */
#define BUTTON_PIN  PINC            /* PINx - register for button input */
#define BUTTON_BIT  PC2             /* bit for button input/output */

#define LED_PORT PORTD              /* PORTx - register for LED output */
#define LED_DDR  DDRD               /* LED data direction register */
#define LED_BIT  PD4                /* bit for button input/output */

//-----------------------------------------------------------------------------------------------
int set_heat(void);
//----------------------------------------------------------------------------------------------- 
int main(void){
															
			//disply_init(); 
			//interrupt_init();                        //Initializations
				
            //DDRC   =0b00000000;
            //PORTC  =0b00000100;
			/*
						DDRC  &= ~(1 << PC2);               // set PC2 as input
						PORTC |=  (1 << PC2);
						
						DDRD  |=  (1 << PD4);                 // set PD4 as output
						PORTD |=  (1 << PD4);
			*/			
						 /* set LED pin as digital output */
						 LED_DDR = 0xff;
						 /* led is OFF initially (set pin high) */
						 LED_PORT |= 0b00000000;
						 /* turn on internal pull-up resistor for the switch */
						 BUTTON_PORT |= 0b00000100;
			
			/* 		
		    //set_sleep_mode(SLEEP_MODE_IDLE);      // Use IDLE sleep mode
			//set_sleep_mode(SLEEP_MODE_ADC);
			
			sei();			                       //Set Global Interrupt
            */
						while(1){	
							
									//beep();
									//blink();
									//start_flag = 1;
									
									 /* the button is pressed when BUTTON_BIT is clear */
									 if ( BUTTON_BIT !=1)
									 {
										//T1_flag     = 1;
										//T2_flag     = 0;
										//start_flag  = 0;
										
										PORTD ^= (1 << PD4);  // toggle LED
										_delay_ms(100);
										
									    }
									/*	
									if (sw2==0){
										T1_flag     = 0;
										T2_flag     = 1;
										start_flag  = 0;
									    }
										*/

									//-------------------------------------------------------------
									/*
									while(start_flag == 0){
											set_heat();
											for(int i=0;i<T1;i++){
												PORTD |=(1<<PD4);
												_delay_ms(100);
												PORTD &=~(1<<PD4);
												_delay_ms(100);
												}												
											if(start_flag == 1)
											T1_flag     = 0;
											T2_flag     = 0; 
											break;
									    }
										*/
									//--------------------------------------------------------------
									/*
									while (start_flag == 1){ 
									       int    channel;
									       int    data_channel[2];                           // save data to Array	
										   
										   num_to_digits(T1, num_digits);                    // Update the mili_volt to display on 7segment
										   _delay_ms(2000);
										   
										   num_to_digits(T2, num_digits);                    // Update the mili_volt to display on 7segment
										   _delay_ms(2000);  
										                       
											for(channel=0;channel<2;channel++){            //set ADC channel : channel must be 0 to 7  (ADC0....ADC7)

													adc_value   = sensor_read(channel);
													_delay_ms(1);
												
													data_channel[channel] = adc_value * 4.6099925;
													ADMUX=0x00;
												   _delay_ms(1);
								
												   }
											   
													   mili_volt_0  =  (data_channel[0]);
													   mili_volt_1  =  (data_channel[1]);
											
													   set_Relay1(mili_volt_0,T1);            //Update relay,led,buzzer status
													   _delay_ms(1);
													   set_Relay2(mili_volt_1,T2);            //Update relay,led,buzzer status
													   _delay_ms(1);
											  
													   num_to_digits(mili_volt_0, num_digits);         // Update the mili_volt to display on 7segment
													   _delay_ms(1000);
											   
													   num_to_digits(mili_volt_1, num_digits);         // Update the mili_volt to display on 7segment
													   _delay_ms(1000);
													   
													if(sw1 == 0 | sw2 == 0)
													start_flag = 0;
													break;
													   
									                 }
													 */
											 
								    //--------------------------------------------------------------	   										   
										//sleep_mode(); 		                               // sleep until next (timer) interrupt, to save power 
						}
    return 0;
}
//-------------------------------------------------------------------------------------------------------------
int set_heat(void){
				  
			   if(start_flag==0 && T1_flag==1 && up==0){
						T1 +=10;
						if(up != 0)
						T1_flag    = 0;
						start_flag = 1;
						//break;    						 
						}
						
			   if(start_flag==0 && T1_flag==1 && down==0){
					    T1 -=10;
						if(down != 0)
						T1_flag    = 0;
						start_flag = 1;
						//break;
					    }
						 
			   if(start_flag==0 && T2_flag==1 && up==0){
					    T2 +=10;
						if(up != 0)
						T2_flag    = 0;
						start_flag = 1;
						//break;
						}
						
			   if(start_flag==0 && T2_flag==1 && down==0){
						T2 -=10;
						if(up != 0)
						T2_flag    = 0;
						start_flag = 1;
						//break;
					    } 
	  
return T1,T2;					  
}
//-------------------------------------------------------------------------------------------------------------
		
//-------------------------------------------------------------------------------------------------------------		
/*
int debounce(void) {
	
			DDRC  &= ~(1 << PC2);               // set PD2 as input
			DDRD  |=  (1 << PD4);                // set PD4 as output
            PORTC |=  (1 << PC2);
			PORTD &= ~(1 << PD4);
			
			while(start_flag==1) {
				
				if(PINC & (1<<PC2)) {       // if button is pressed
					_delay_ms(20);            // debounce delay
					
					if(sw1==0) {   // check if button is still pressed
						PORTD ^= (1 << PD4);  // toggle LED
					  }
				  }
			  }

			return 0;
}
*/
//-------------------------------------------------------------------------------------------------------------


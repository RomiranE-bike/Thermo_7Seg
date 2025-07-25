/*
 * adc+7seg+interrupt.c
 * ver.4.05
 * Created: 1/29/2024 11:59:40
 * edited : me
 */ 
//-------------------------------------------------------------------------------------------------------------
#include <avr/io.h>
//#include <avr/interrupt.h>
#include <util/delay.h>
//#include <avr/sleep.h>
//#include <avr/wdt.h>
#include "adc.h"
#include "beep.h"
#include "display.h"
//-------------------------------------------------------------------------------------------------------------
#define button_PORT    PORTC
#define button_DDR     DDRC
#define button_PIN     PINC                                

#define RUN_SW         PINC3
#define SETUP_SW       PINC4

#define action_DDR     DDRD
#define action_PORT    PORTD
#define RL1_BIT        PD6// bit for RL1 output
#define RL2_BIT        PD7 // bit for RL2 output
//-------------------------------------------------------------------------------------------------------------
unsigned char sens_value[3]; 
//-------------------------------------------------------------------------------------------------------------
volatile unsigned int T1_value;
volatile unsigned int T2_value;
volatile unsigned int sens_value_1;
volatile unsigned int sens_value_2;
volatile unsigned int button_value; 
//-------------------------------------------------------------------------------------------------------------
int up_SW_flag_1;
int down_SW_flag_1;
int up_SW_flag_2;
int down_SW_flag_2;

int RUN_flag; 
int SETUP_flag;
//------------------------------------------------------------------------------------------------------------- 
void IO_init(void);
void thermo_setting(void);
void thermo_action(void);
void sensing(void);

void POWER_OFF(void);
//-------------------------------------------------------------------------------------------------------------
//MAIM SECTION CODE
int main(void){					
		
		IO_init();
		action_PORT &= ~ ((1<<PD7) | (1<<PD6) |(1<<PD5) | (1<<PD4));//SET  OFF		 
		while(1){
							
			if(! (button_PIN & 1<<SETUP_SW) ){					
					SETUP_flag = 1;
					while(SETUP_flag == 1){
						display_char(null,P,S,E);
						_delay_ms(10);	
						sensing();
						//display_num(button_value);// Update the button_value to display on 7segment
						//_delay_ms(1000);				
						thermo_setting();
						if(! (button_PIN & 1<<RUN_SW) ){
							SETUP_flag = 0;
							break;
							} //end of if		
							} //end of while	
							} //end of if
				
				
			else{					
					
					sensing();
										
					display_char(null,S,E,1);//sensor value 1
					_delay_ms(5000);
					display_num(sens_value_1);// Update the sens_value_0 to display on 7segment
					_delay_ms(5000);
					display_char(null,S,E,2);//sensor value 2
					_delay_ms(5000);
					display_num(sens_value_2);// Update the sens_value_1 to display on 7segment
					_delay_ms(5000);
														
					//thermo_action();					
					_delay_ms(5000);
					
			            } //end of else								
				   			   	  	 	 		    
			}//end of main while		                     	     						         
    return 0;
}//end of main
//-------------------------------------------------------------------------------------------------------------
//IO INITIALISE
void IO_init(void){
	
	button_DDR  &=~ ( (1<<PC4) | (1<<PC3) ); //POWER_ON_SW and POWER_OFF_SW set as input
	button_PORT |=  ( (1<<PC4) | (1<<PC3) ); //enable internal pull-up resistors 
	
	segment_DDR  = 0XFF; //Port B [7,6,5,3,2,1,0] as [dot,G,F,E,D,C,B,A]
	digit_DDR   |= (1<<PD3) | (1<<PD2) | (1<<PD1) | (1<<PD0);  //Port D[3,2,1,0] as out put
	
	action_DDR  |=  ((1<<PD7) | (1<<PD6) |(1<<PD5) | (1<<PD4)); //Port D[7,6,5,4] as output
	action_PORT &= ~ ((1<<PD7) | (1<<PD6) |(1<<PD5) | (1<<PD4));//SET  OFF	
}
//-------------------------------------------------------------------------------------------------------------
//Interrupt Vectors in ATmega8 page 44
void POWER_OFF(void){
	
	cli();// Disable interrupts before next commands
	wdt_disable();// Disable watch dog timer to save power
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);// Set sleep mode power down
	//set_sleep_mode(SLEEP_MODE_IDLE);// Use IDLE sleep mode
	//set_sleep_mode(SLEEP_MODE_ADC);
	sleep_enable();
	//sleep_bod_disable(); // Disable brown-out detector
	sei(); // Enable interrupts
	sleep_cpu();
	sleep_disable();
}
//-------------------------------------------------------------------------------------------------------------
void sensing(void){	
	
		int    channel; 
		int    adc_value;                        											                       
		for(channel=0;channel<3;channel++){//set ADC channel : channel must be 0 to 7  (ADC0....ADC7)
			    sens_value[channel]=0;
				adc_value   = sensor_read(channel);
				_delay_ms(10);											
				sens_value[channel] = adc_value ;//* 0.46099925;
				ADMUX=0x00;
				_delay_ms(10);								
				}
		sens_value_1  =  (sens_value[0]);
		sens_value_2  =  (sens_value[1]);
		button_value  =  (sens_value[2]);
		thermo_action();																					   															   												 
}
//-------------------------------------------------------------------------------------------------------------
// Relay 's action by increase or decrease  temperature
void thermo_action(void){
	if(sens_value_1 < T1_value){
		action_PORT = (1<<RL1_BIT) ;//Relay 1 on to increase temperature
	}//end of if
	else if(sens_value_1 > (T1_value)){
		action_PORT =~ (1<<RL1_BIT);//Relay 1 off to decrease temperature
	}//end of if

    if(sens_value_2 < T2_value){
		action_PORT = (1<<RL2_BIT) ;//Relay 2 on to increase temperature
	}//end of if
	else if(sens_value_2 > (T2_value)){
		action_PORT =~ (1<<RL2_BIT);//Relay 2 off to decrease temperature
	}//end of if
}
//-------------------------------------------------------------------------------------------------------------
//Setting Thermo offset //
void thermo_setting(void){
 	
	while( button_value == 196 ){ //  thermo_set_SW1 is activated
		sensing();
		T1_value +=1;
		display_num(T1_value);
		_delay_ms(1);
		if(T1_value>999){
			T1_value =0;
		}//end of if
	}//end of if
	
	while( button_value == 194 ){ //  thermo_set_SW1 is activated
		sensing();
		T1_value -=1;
		display_num(T1_value);
		_delay_ms(1);
		if(T1_value<0){
			T1_value =999;
		}//end of if
	}//end of if
	
	while( button_value == 122){ //  thermo_set_SW1 is activated
		sensing();
		T2_value +=1;
		display_num(T2_value);
		_delay_ms(1);
		if(T2_value>999){
			T2_value =0;
		}//end of if
	}//end of if
	while( button_value == 83 ){ //  thermo_set_SW1 is activated
		sensing();
		T2_value -=1;
		display_num(T2_value);
		_delay_ms(1);
		if(T2_value<0){
			T2_value =999;
		}//end of if
	}//end of if
}	
//---------------------------------------------------------------------------------	
	/*
			display_char(null,P,S,E);
		
			while( !(set_SW_PIN &  1<<T1_UP_SW )){ //  thermo_set_SW1 is activated
				thermo_set_value1 +=1;
				display_num(thermo_set_value1);
				_delay_ms(1);
				if(thermo_set_value1>999){
						thermo_set_value1 =0;
					}//end of if	
				}//end of if
				}

			if( !(set_SW_PIN & 1<<T1_DOWN_SW) ){ //  thermo_set_SW1 is activated
					thermo_set_value1 -=1;
					display_num(thermo_set_value1);
					_delay_ms(1);
					if(thermo_set_value1<0){
						thermo_set_value1 =999;
					}//end of if
				}//end of if
		
			if( !(set_SW_PIN & 1<<T2_UP_SW) ){ //  thermo_set_SW1 is activated
					thermo_set_value2 +=1;
					display_num(thermo_set_value2);
					_delay_ms(1);
					if(thermo_set_value2>999){
						thermo_set_value2 =0;
					}//end of if
				}//end of if
			if( !(set_SW_PIN & 1<<T2_DOWN_SW) ){ //  thermo_set_SW1 is activated
					thermo_set_value2 -=1;
					display_num(thermo_set_value2);
					_delay_ms(1);
					if(thermo_set_value2<0){
						thermo_set_value2 =999;
					}//end of if
				}//end of if
}
*/
//---------------------------------------------------------------------------------		
	
//---------------------------------------------------------------------------------
/*
 void thermo_setting(void){	 	 	 	 
			  				 			  
			if(button_value == 159 ){ //up_SW1 is activated
				       up_SW_flag_1 = 1;
					   display_char(null,S,E,1);
					   _delay_ms(1);
					   display_char(null,0,U,P);
					   _delay_ms(1);
					   while(up_SW_flag_1 ==1){
					   thermo_set_value1 ++;
						display_num(thermo_set_value1);
						_delay_ms(1);
						if(thermo_set_value1>999){
							   thermo_set_value1 =0;
						       }//end of if
						if(! (button_PIN & 1<<SETUP_SW) ){
							up_SW_flag_1 = 0;
							break;
						       }//end of if						   
				          }//end of while
						}//end of if	
							  
		   if(button_value == 194){//down_SW1 is activated
			             down_SW_flag_1 = 1;
				         display_char(null,S,E,1);
				         _delay_ms(100);
				         display_char(null,d,O,n);
				         _delay_ms(100);
						 while(down_SW_flag_1 ==1){
							 thermo_set_value1 --;
							 display_num(thermo_set_value1);
							 _delay_ms(1);
							 if(thermo_set_value1<0){
								 thermo_set_value1 =999;
							     }//end of if
							 if(! (button_PIN & 1<<SETUP_SW) ){
								 down_SW_flag_1 = 0;
								 break;
							     }//end of if
						 }//end of while				
					}//end of if
							 
		   if(button_value == 122){//up_SW2 is activated
						 up_SW_flag_2 = 1;
						 display_char(null,S,E,2);
						 _delay_ms(1);
						 display_char(null,null,U,P);
						 _delay_ms(1);
						 while(up_SW_flag_2 ==1){
							 thermo_set_value2 ++;
							 display_num(thermo_set_value2);
							 _delay_ms(1);
							 if(thermo_set_value1>999){
								 thermo_set_value2 =0;
							 }//end of if
							 if(! (button_PIN & 1<<SETUP_SW) ){
								 up_SW_flag_2 = 0;
								 break;
							 }//end of if
						 }//end of while
					 }//end of if
							  
		  if(button_value == 83 ){//down_SW2 is activated
						down_SW_flag_2 = 1;
						display_char(null,S,E,2);
						_delay_ms(100);
						display_char(null,d,O,n);
						_delay_ms(100);
						while(down_SW_flag_2 ==1){
							thermo_set_value2 --;
							display_num(thermo_set_value2);
							_delay_ms(1);
							if(thermo_set_value2<0){
								thermo_set_value2 =999;
							}//end of if
							if(! (button_PIN & 1<<SETUP_SW) ){
								down_SW_flag_2 = 0;
								break;
							}//end of if
						}//end of while
					}//end of if	
					  
}
*/
//-------------------------------------------------------------------------------------------------------------



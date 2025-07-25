/*
 * adc+7seg+interrupt.c
 * ver.4.07
 * Created: 2/3/2024 11:59:40
 * edited : me
 */ 
//-------------------------------------------------------------------------------------------------------------
#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

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
#define RL1_BIT        PD4// bit for RL1 output
#define RL2_BIT        PD5 // bit for RL2 output
#define display_delay  2000
//-------------------------------------------------------------------------------------------------------------
int sens_value[3]; 
//-------------------------------------------------------------------------------------------------------------
int T1_limit =25;
int T2_limit =25;

int sens_value_1;
int sens_value_2;
int button_value; 

int    T1_sens;
int    T2_sens;
int    SW_button;

int    RL1_status;
int    RL2_status;

int adress_1 = 4;
int adress_2 = 8;
//-------------------------------------------------------------------------------------------------------------
int up_SW_flag_1;
int down_SW_flag_1;
int up_SW_flag_2;
int down_SW_flag_2;

int RL1_flag;
int RL2_flag;

int RUN_flag; 
int SETUP_flag;
//------------------------------------------------------------------------------------------------------------- 
void IO_init(void);
void thermo_setting(void);
int thermo_action(void);
int sensing(void);
//-------------------------------------------------------------------------------------------------------------
//MAIM SECTION CODE
int main(void){					
		
		IO_init();	
		beep();
		blink();	 
		while(1){
										
			if(! (button_PIN & 1<<SETUP_SW) ){					
					SETUP_flag = 1;
					beep();
					while(SETUP_flag == 1){
						moving_display(null,P,S,E);
						sensing();
						thermo_setting();						
						if(! (button_PIN & 1<<RUN_SW) ){
							beep();
							SETUP_flag = 0;
							break;
							} //end of if		
					} //end of while	
			} //end of if			
			else{
				sensing();
				thermo_action();
				message(T1_sens,1,RL1_status);
				message(T2_sens,2,RL2_status);
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
int sensing(void){	
	
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
		button_value  =  (sens_value[2]);//button = (button_value *5+512)/1024
		
		T1_sens = (sens_value_1 * 4.609)/1;
		T2_sens = (sens_value_2 * 4.609)/1;		
		//SW_button = button_value * 0.46099925;
		
		return T1_sens,T2_sens;
																											   															   												 
}
//-------------------------------------------------------------------------------------------------------------
// Relay 's action by increase or decrease  temperature
int thermo_action(void){

	T1_limit =  ((eeprom_read_word((uint16_t*)adress_1)) * 0.46099925)/1;
	T2_limit =  ((eeprom_read_word((uint16_t*)adress_2)) * 0.46099925)/1;
	
	if(T1_sens < T1_limit && T2_sens < T2_limit){
		   RL1_flag = 1;
		   RL2_flag = 1; 	 
		   if(RL1_flag & RL2_flag){			   
			action_PORT |= (1<<RL1_BIT);//Relay 1 on to increase temperature
			action_PORT |= (1<<RL2_BIT);//Relay 2 on to increase temperature
			RL1_status =1;
			RL2_status =1;
		    }//end of if
			}//end of if			
		   
	if((T1_sens > T1_limit || T1_sens == T1_limit) && (T2_sens > T2_limit || T2_sens == T2_limit)){
			RL1_flag = 0;
			RL2_flag = 0;	   
		   if(!(RL1_flag & RL2_flag)){
			   action_PORT &=~ (1<<RL1_BIT);//Relay 1 off to decrease temperature
			   action_PORT &=~ (1<<RL2_BIT);//Relay 2 off to decrease temperature
			   RL1_status =0;
			   RL2_status =0;
		       }//end of if
			   }//end of if
		   
	if((T1_sens < T1_limit) && (T2_sens > T2_limit || T2_sens == T2_limit)){
			RL1_flag = 1;
			RL2_flag = 0;
			if( (RL1_flag & (~(RL2_flag))) ){
				action_PORT |= (1<<RL1_BIT);//Relay 1 on to decrease temperature
				action_PORT &=~ (1<<RL2_BIT);//Relay 2 off to decrease temperature
				RL1_status =1;
				RL2_status =0;
			    }//end of if
				}//end of if
				
	if((T1_sens > T1_limit || T1_sens == T1_limit) && (T2_sens < T2_limit)){
			RL1_flag = 0;
			RL2_flag = 1;
			if( ((~(RL1_flag)) & RL2_flag)){
				action_PORT &=~ (1<<RL1_BIT);//Relay 1 off to decrease temperature
				action_PORT |= (1<<RL2_BIT);//Relay  2 on to decrease temperature
				RL1_status =0;
				RL2_status =1;
			    }//end of if
				}//end of if	
				
	return  RL1_status,RL2_status;							   		   		   
}
//-------------------------------------------------------------------------------------------------------------
//Setting Thermo offset //
void thermo_setting(void){

	while( button_value >220  &&  button_value<235   ){ //  thermo_set_SW1 is activated
		sensing();		
		if(T1_limit<999){
			T1_limit ++;
			display_num(T1_limit);
			eeprom_write_word((uint16_t*)adress_1,T1_limit); //write memory
		}//end of if
		else{
			T2_limit=0;
			break;
		}					
	    }//end of while
	
	while( button_value >240  &&  button_value<250 ){ //  thermo_set_SW1 is activated
		sensing();
		if(T1_limit>0){
			T1_limit --;
			display_num(T1_limit);
			eeprom_write_word((uint16_t*)adress_1,T1_limit); //write memory
		}//end of if
		else{
			T2_limit=999;
			break;
		}
	    }//end of while
	
	while( button_value >170  &&  button_value<200){ //  thermo_set_SW1 is activated
		sensing();
		if(T2_limit<999){
			T2_limit ++;
			display_num(T2_limit);
			eeprom_write_word((uint16_t*)adress_2,T2_limit); //write memory
		}//end of if
		else{
			T2_limit=0;
			break;
		}
	    }//end of while
	while( button_value >70  &&  button_value<100 ){ //  thermo_set_SW1 is activated
		sensing();
		if(T2_limit>0){
			T2_limit --;
			display_num(T2_limit);
			eeprom_write_word((uint16_t*)adress_2,T2_limit); //write memory
		}//end of if
		else{
			T2_limit=999;
			break;					
			}
	    }//end of while
		
}	 
//-------------------------------------------------------------------------------------------------------------
/*
#define INPUT_PIN A0

void setup() {
	Serial.begin(9600);
	pinMode(INPUT_PIN, INPUT);
}

void loop() {
	int result = readAnalogButton();
	Serial.println(result);
}

int readAnalogButton() {
	int button = analogRead(INPUT_PIN);
	if (button > 921) return 0;
	if (button < 256) return 1;
	if (button < 598) return 2;
	if (button < 726) return 3;
	if (button < 794) return 4;
	if (button < 921) return 5;
}

#define BUTTONS 5
#define RESOLUTION 1023

int readAnalogButton() {
	float avg = RESOLUTION / float(BUTTONS);
	int val = analogRead(ANALOG_PIN);

	if (val > (BUTTONS - 0.5) * avg) {
		return 0;
	}

	for (int i = 0; i < BUTTONS; i++) {
		if (val < round((i + 0.5) * avg)) {
			return i + 1;
		}
	}

	return 0;
}

*/
//-------------------------------------------------------------------------------------------------------------
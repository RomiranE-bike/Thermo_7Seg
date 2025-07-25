/*
 * adc+7seg+interrupt.c
 * ver.10.02 final ok
 * Created: 2/23/2024 20:47:40
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
#include "NTC.h"
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
uint16_t sens_value[3]; 
//-------------------------------------------------------------------------------------------------------------
uint16_t T1_limit;
uint16_t T2_limit;

uint16_t eeprom_value_1;
uint16_t eeprom_value_2;

uint16_t sens_value_1;
uint16_t sens_value_2;
uint16_t button_value; 

uint16_t    T1_sens;
uint16_t    T2_sens;
uint16_t    ladder_SW_button;

uint16_t    RL1_status;
uint16_t    RL2_status;

uint16_t adress_1 = 4;
uint16_t adress_2 = 8;
//-------------------------------------------------------------------------------------------------------------
uint16_t up_SW_flag_1;
uint16_t down_SW_flag_1;
uint16_t up_SW_flag_2;
uint16_t down_SW_flag_2;

uint16_t RL1_flag;
uint16_t RL2_flag;

uint16_t RUN_flag; 
uint16_t SETUP_flag;
//------------------------------------------------------------------------------------------------------------- 
void IO_init(void);
void sensing(void);
//void thermo_setting(uint16_t eeprom_value_1,uint16_t eeprom_value_2);
//uint16_t thermo_action(uint16_t T1_sens,uint16_t T2_sens,uint16_t T1_limit,uint16_t T2_limit);
void thermo_setting(void);
void thermo_action(void);
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
					display_char(null,p,s,e);
					thermo_setting();
												
					if(! (button_PIN & 1<<RUN_SW) ){
						beep();
						SETUP_flag = 0;
						break;
						} //end of if		
					    } //end of while	
			            } //end of if			
				else{
						thermo_action();										
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
void sensing(void){	
	
		uint16_t    channel; 
		uint16_t    adc_value;
		                        											                       
			for(channel=0;channel<3;channel++){//set ADC channel : channel must be 0 to 7  (ADC0....ADC7)
					sens_value[channel]=0;
					adc_value   = sensor_read(channel);
					_delay_ms(10);											
					sens_value[channel] = adc_value ;//* 4.6099925  convert volt to m_volt
					ADMUX=0x00;
					_delay_ms(10);								
					}
			sens_value_1  =  (sens_value[0]);
			sens_value_2  =  (sens_value[1]);
			button_value  =  (sens_value[2])/10;//button {22,50,68,84} for sensing ladder resistor.
		
			//ladder_SW_button = button_value * 4.6099925;																											   															   												 
}
//-------------------------------------------------------------------------------------------------------------
// Relay 's action by increase or decrease  temperature
//uint16_t thermo_action(uint16_t T1_sens,uint16_t T2_sens,uint16_t T1_limit,uint16_t T2_limit){
void thermo_action(void){
		
	//R1=100K,R2=150 :GAIN=(R1+R2)/R2  VO=VI*(1+(R1/R2)) >>> GAIN=(100K+150)/150 =667.6666666
	T1_limit =(eeprom_read_word((uint16_t*)adress_1));	
	T2_limit =(eeprom_read_word((uint16_t*)adress_2));
		
	
	sensing();
	
	T1_sens = NTC_function_1(sens_value_1);
	T2_sens = NTC_function_1(sens_value_2);
	
	
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
				
	display_char(null,c,h,1);//display_char(uint16_t ch3,uint16_t ch2,uint16_t ch1,uint16_t ch0)
	_delay_ms(2000);
	display_num(T1_sens,250);//display_num(uint16_t number,uint16_t cycle)
	
	display_char(null,c,h,2);//display_char(uint16_t ch3,uint16_t ch2,uint16_t ch1,uint16_t ch0)
	_delay_ms(2000);
	display_num(T2_sens,250);//display_num(uint16_t number,uint16_t cycle)
					   		   		   
}
//-------------------------------------------------------------------------------------------------------------
//Setting Thermo offset //
//void thermo_setting(uint16_t eeprom_value_1,uint16_t eeprom_value_2){
void thermo_setting(void){
	eeprom_value_1 = (eeprom_read_word((uint16_t*)adress_1));
	eeprom_value_2 = (eeprom_read_word((uint16_t*)adress_2));
	sensing();	
	while( button_value >20  &&  button_value<25  ){ // 22 thermo_set_SW1 is activated
		sensing();
		eeprom_value_1 ++;
		display_num(eeprom_value_1,50);
		_delay_ms(10);
		eeprom_write_word((uint16_t*)adress_1,eeprom_value_1); //write memory
		_delay_ms(10);
		if(eeprom_value_1>999){
			eeprom_value_1 = 0;
		}//end of if
		if(!(button_value >20  &&  button_value<25)){
			beep();
			break;
		   }//end of if
	    }//end of while
	
	while( button_value >25  &&  button_value<55 ){ // 50 thermo_set_SW1 is activated
		sensing();
		eeprom_value_1 --;
		display_num(eeprom_value_1,50);
		_delay_ms(10);
		eeprom_write_word((uint16_t*)adress_1,eeprom_value_1); //write memory
		_delay_ms(10);
		if(eeprom_value_1<1){
			eeprom_value_1 =999;
		}//end of if
		if(!(button_value >25  &&  button_value<55)){
			beep();
			break;
		}//end of if
	    }//end of while
	
	while( button_value >55  &&  button_value<70){ // 68 thermo_set_SW1 is activated
		sensing();
		eeprom_value_2 ++;
		display_num(eeprom_value_2,50);
		_delay_ms(10);
		eeprom_write_word((uint16_t*)adress_2,eeprom_value_2); //write memory
		_delay_ms(10);
		if(eeprom_value_2>999){
			eeprom_value_2 = 0;
		}//end of if
		if(!(button_value >55  &&  button_value<70)){
			beep();
			break;
		}//end of if
	    }//end of while
	
	while( button_value >70  &&  button_value<90 ){ //  84 thermo_set_SW1 is activated
		sensing();
		eeprom_value_2 --;
		display_num(eeprom_value_2,50);
		_delay_ms(10);
		eeprom_write_word((uint16_t*)adress_2,eeprom_value_2); //write memory
		_delay_ms(10);
		if(eeprom_value_2<1){
			eeprom_value_2 = 999;
		}//end of if
		if(!(button_value >70  &&  button_value<90)){
			beep();
			break;
		}//end of if
	    }//end of while
}	 
//-------------------------------------------------------------------------------------------------------------

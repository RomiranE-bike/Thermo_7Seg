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
unsigned char sens_value[3]; 
//-------------------------------------------------------------------------------------------------------------
volatile unsigned int T1_value;
volatile unsigned int T2_value;
volatile unsigned int sens_value_1;
volatile unsigned int sens_value_2;
volatile unsigned int button_value; 
unsigned int adress_1 = 4;
unsigned int adress_2 = 8;
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
void thermo_action(void);
void sensing(void);
void POWER_OFF(void);
void message(int sens_value,int num,int state);
void alarm(void);
//-------------------------------------------------------------------------------------------------------------
//MAIM SECTION CODE
int main(void){					
		
		IO_init();	
		beep();
		blink();	 
		while(1){
			
			moving_display(S,E,n);
			//spelit_num(478);
			//moving_display(ch2,ch1,ch0);
			display_num(487);
					/*		
			if(! (button_PIN & 1<<SETUP_SW) ){					
					SETUP_flag = 1;
					beep();
					while(SETUP_flag == 1){
						display_char(null,P,E,E);
						_delay_ms(100);
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
				blink();
				sensing();
				thermo_action();
				} //end of else	
				*/											   			   	  	 	 		    
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
		_delay_ms(100);																											   															   												 
}
//-------------------------------------------------------------------------------------------------------------
// Relay 's action by increase or decrease  temperature
void thermo_action(void){

	T1_value =  eeprom_read_word((uint16_t*)adress_1);
	T2_value =  eeprom_read_word((uint16_t*)adress_2);
	
	if(sens_value_1 < T1_value){
		if(sens_value_2 < T1_value){
		   RL1_flag = 1;
		   RL2_flag = 1; 
		   //beep();
		   blink;	 
		   if(RL1_flag & RL2_flag){			   
			action_PORT |= (1<<RL1_BIT);//Relay 1 on to increase temperature
			message(sens_value_1,1,1);
			action_PORT |= (1<<RL2_BIT);//Relay 2 on to increase temperature
			message(sens_value_2,2,1); 
		    }//end of if
			}//end of if
			}//end of if
		   
	if(sens_value_1 > T1_value || sens_value_1 == T1_value){
		if(sens_value_2 > T1_value || sens_value_2 == T1_value){
			RL1_flag = 0;
			RL2_flag = 0;
			//beep();
			blink();	   
		   if(!(RL1_flag & RL2_flag)){
			   action_PORT &=~ (1<<RL1_BIT);//Relay 1 off to decrease temperature
			   message(sens_value_1,1,0);
			   action_PORT &=~ (1<<RL2_BIT);//Relay 2 off to decrease temperature
			   message(sens_value_2,2,0);
		       }//end of if
			   }//end of if
			   }//end of if
		   
	if(sens_value_1 < T1_value){
		if(sens_value_2 > T1_value || sens_value_2 == T1_value){
			RL1_flag = 1;
			RL2_flag = 0;
			//beep();
			blink();
			if( (RL1_flag & (~(RL2_flag))) ){
				action_PORT |= (1<<RL1_BIT);//Relay 1 on to decrease temperature
				message(sens_value_1,1,1);
				action_PORT &=~ (1<<RL2_BIT);//Relay 2 off to decrease temperature
				message(sens_value_2,2,0);
			    }//end of if
				}//end of if
				}//end of if
				
	if(sens_value_1 > T1_value || sens_value_1 == T1_value){
		if(sens_value_2 < T1_value){
			RL1_flag = 0;
			RL2_flag = 1;
			//beep();
			blink();
			if( ((~(RL1_flag)) & RL2_flag)){
				action_PORT &=~ (1<<RL1_BIT);//Relay 1 off to decrease temperature
				message(sens_value_1,1,0);
				action_PORT |= (1<<RL2_BIT);//Relay  2 on to decrease temperature
				message(sens_value_2,2,1);
			    }//end of if
				}//end of if
				}//end of if					   		   
		   
}
//-------------------------------------------------------------------------------------------------------------
//Setting Thermo offset //
void thermo_setting(void){
    blink();
	while( button_value >220  &&  button_value<235   ){ //  thermo_set_SW1 is activated
		sensing();
		T1_value +=1;
		display_num(T1_value);
		_delay_ms(1);
		eeprom_write_word((uint16_t*)adress_1,T1_value); //write memory		
		if(T1_value>999){
			T1_value =0;
		}//end of if
	}//end of if
	
	while( button_value >240  &&  button_value<250 ){ //  thermo_set_SW1 is activated
		sensing();
		T1_value -=1;
		display_num(T1_value);
		_delay_ms(1);
		eeprom_write_word((uint16_t*)adress_1,T1_value); //write memory
		if(T1_value<0){
			T1_value =999;
		}//end of if
	}//end of if
	
	while( button_value >170  &&  button_value<200){ //  thermo_set_SW1 is activated
		sensing();
		T2_value +=1;
		display_num(T2_value);
		_delay_ms(1);
		eeprom_write_word((uint16_t*)adress_2,T2_value); //write memory
		if(T2_value>999){
			T2_value =0;
		}//end of if
	}//end of if
	while( button_value >70  &&  button_value<100 ){ //  thermo_set_SW1 is activated
		sensing();
		T2_value -=1;
		display_num(T2_value);
		_delay_ms(1);
		eeprom_write_word((uint16_t*)adress_2,T2_value); //write memory
		if(T2_value<0){
			T2_value =999;
		}//end of if
	}//end of if
		
}	 
//-------------------------------------------------------------------------------------------------------------
void message(int sens_value,int num,int state){
	display_char(null,null,c,num);//sensor value 1
	_delay_ms(display_delay);
	display_num(sens_value);
	_delay_ms(display_delay);
	//display_char(null,L,num,state);
	//_delay_ms(display_delay);		
}
//-------------------------------------------------------------------------------------------------------------
void alarm(void){
	for(int i=0;i<50;i++){
		
		_delay_ms(display_delay);
		beep();
		blink();
	}
}
//-------------------------------------------------------------------------------------------------------------
/*
void eeprom_update(int eep_value_1,int eep_value_2){
	 
	display_char(null,E,E,P);
	_delay_ms(display_delay);
	uint16_t  read_value_1;
	uint16_t  read_value_2;
	uint16_t  eeprom_value_1 = eep_value_1;
	uint16_t  eeprom_value_2 = eep_value_2;
	
	eeprom_write_word((uint16_t*)500,eeprom_value_1); //write memory
	eeprom_write_word((uint16_t*)600,eeprom_value_2); //write memory
	
	//eeprom_update_word((uint16_t*)500,eeprom_value_1); //update memory
	//eeprom_update_word((uint16_t*)500,eeprom_value_2); //update memory
	
	read_value_1 = eeprom_read_word((uint16_t*)500); //read the value stored in the memory
	read_value_2 = eeprom_read_word((uint16_t*)600); //read the value stored in the memory
	
	display_char(null,E,E,1);
	_delay_ms(display_delay);
	display_num(read_value_1); //PASS THE VALUE OF 'eeprom_value' FOR DIGIT BREAKUP AND DISPLAY
	_delay_ms(display_delay);
	display_char(null,E,E,2);
	_delay_ms(display_delay);
	display_num(read_value_2); //PASS THE VALUE OF 'eeprom_value' FOR DIGIT BREAKUP AND DISPLAY
	_delay_ms(display_delay);
}
*/
//-------------------------------------------------------------------------------------------------------------
/*
	unsigned char s;
	unsigned char eeprom_var[] EEMEM; // We define a variable in EEPROM
	s = eeprom_read_byte(&eeprom_var); //read the byte from the EEPROM and place it in "s"
	eeprom_write_byte(&eeprom_var, s); // write "s" to EEPROM 
	
	char myEepromString[] EEMEM = "Hello World!";
	
	#define ADDRESS_1 46  // This could be anything from 0 to the highest EEPROM address
	#define ADDRESS_2 52  // This could be anything from 0 to the highest EEPROM address
	#define ADDRESS_3 68  // This could be anything from 0 to the highest EEPROM address
	uint8_t dataByte1 = 0x7F;  // Data for address 1
	uint8_t dataByte2 = 0x33;  // Data for address 2
	uint8_t dataByte3 = 0xCE;  // Data for address 3
	
	eeprom_update_byte((uint8_t*)ADDRESS_1, dataByte1);

	uint16_t c = 0;
	eeprom_write_word((uint16_t*)10 ,c);

	// check if value has already been used/initialized...
	if (eeprom_read_word((uint16_t*)10) == UINT16_MAX){
		// ...if not, initial to 0
		eeprom_write_word((uint16_t*)10 , 0);
	         }
	else{
		// ...if yes, read value back
		c = eeprom_read_word((uint16_t*)10);
	     }
	*/
//-------------------------------------------------------------------------------------------------------------
/*
#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>

EEMEM unsigned char colors[2][3]={{1, 2, 3},{4, 5, 6}};

eeprom_write_byte(&colors[0][0], 1); 
eeprom_write_byte(&colors[0][1], 2);
eeprom_write_byte(&colors[0][2], 3);
eeprom_write_byte(&colors[1][0], 4);
eeprom_write_byte(&colors[1][1], 5);
eeprom_write_byte(&colors[1][2], 6);

unsigned char temp;
temp = eeprom_read_byte(&colors[1][0]); 
*/
//-------------------------------------------------------------------------------------------------------------
/*
void eeprom_update_word (uint16_t *p, uint16_t  value);
 //Update a word value to EEPROM address referred by the pointer p
 
void eeprom_write_byte(uint8_t *p, uint8_t value); 
//Write a byte value to EEPROM address referred by the pointer p

void eeprom_write_word(uint16_t *p, uint16_t value);
//Write a word value to EEPROM referred by the pointer p

uint16_t eeprom_read_word(const uint16_t *p);
//Read one 16-bit word from EEPROM address referred by the pointer p

uint8_t eeprom_read_byte(const uint8_t *p);
//Returns one byte from EEPROM address referred by the pointer p

void eeprom_is_ready(void); 
//Returns 1 if EEPROM is ready for a new read/write operation, 0 if no

uint8_t eeprom_read_byte (const uint8_t *p);
//The function returns one data byte which is stored in the EEPROM address referred by the pointer p
//The pointer refers to the EEPROM address from which the data byte need to be read
*/
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
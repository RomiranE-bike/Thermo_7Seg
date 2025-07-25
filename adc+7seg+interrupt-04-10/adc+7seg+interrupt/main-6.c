/*
 * adc+7seg+interrupt.c
 * ver.4.05
 * Created: 1/29/2024 11:59:40
 * edited : me
 */ 
//-------------------------------------------------------------------------------------------------------------
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include "adc.h"
#include "beep.h"
//-------------------------------------------------------------------------------------------------------------
#define segment_PORT   PORTB
#define segment_DDR    DDRB

#define digit_PORT     PORTD
#define digit_DDR      DDRD

#define output_DDR     PORTD
//-------------------------------------------------------------------------------------------------------------
#define LED_DDR  DDRD// LED data direction register 
#define LED_PORT PORTD// PORTx - register for LED output 
#define LED_BIT  PD4// bit for LED output 

#define BUZ_DDR  DDRD// LED data direction register
#define BUZ_PORT PORTD// PORTx - register for LED output
#define BUZ_BIT  PD5// bit for LED output

#define RL1_DDR  DDRD// RL1 data direction register
#define RL1_PORT PORTD// PORTx - register for RL1 output
#define RL1_BIT  PD6// bit for RL1 output 

#define RL2_DDR  DDRD// RL2 data direction register
#define RL2_PORT PORTD// PORTx - register for RL2 output
#define RL2_BIT  PD7 // bit for RL2 output
//-------------------------------------------------------------------------------------------------------------
#define A     20  // 0b01110111
#define b     30  // 0b01111100
#define c     40  // 0b00111001
#define d     50  // 0b01011110
#define E     60  // 0b01111001
#define F     70  // 0b01110001
#define H     80  // 0b01110110
#define O     90  // 0b00111111
#define L     100 // 0b00111000
#define n     110 // 0b00110111
#define U     120 // 0b00111110
#define S     130 // 0b01101101
#define t     140 // 0b01111000
#define P     150 // 0b01110011
#define dot   200 // 0b10000000
#define null  210 // 0b00000000
//-------------------------------------------------------------------------------------------------------------
#define button_PORT    PORTC
#define button_DDR     DDRC
#define button_PIN     PINC

#define thermo_set_SW1         1 //ladder resistor  adc voltage  is 0<button_value<2 volt
#define thermo_set_SW2         2 //ladder resistor  adc voltage  is 2<button_value<3 volt
#define up_SW                  3 //ladder resistor  adc voltage  is 3<button_value<4 volt
#define down_SW                4 //ladder resistor  adc voltage  is 4<button_value<5 volt

#define START_SW            PC3
#define SETUP_SW           PC4

#define BUTTON_HOLD_MS   1000  // Press button for 3 second
//-------------------------------------------------------------------------------------------------------------            
unsigned char           segment_buffer[4]; // 4 digit  7segment display
unsigned char           char_buffer[4]; // 4 digit  7segment display
static unsigned char    dot_pos = 3;
unsigned char sens_value[3]; 
//-------------------------------------------------------------------------------------------------------------
int thermo_set_value1 = 270;
int thermo_set_value2 = 150;
int sens_value_1;
int sens_value_2;
int button_value; 
int accept_value = 0;// ' C

int v_refrence;
int adc_value;
int thermo;
int degree;

int T1; // 'C
int T2; // 'C
//-------------------------------------------------------------------------------------------------------------
int thermo_set_SW1_flag;
int thermo_set_SW2_flag;
int up_SW_flag;
int down_SW_flag;

int POWER_SW_timer;// milliseconds counter
int START_flag; 
int SETUP_flag;
//-------------------------------------------------------------------------------------------------------------
int button_status  = 1;
int button_down    = 0;
int button_up      = 1;
//------------------------------------------------------------------------------------------------------------- 
void IO_init(void);
void thermo_setting(void);
void thermo_action(void);
void sensing(void);

void POWER_OFF(void);
//-------------------------------------------------------------------------------------------------------------
int  segment_Code(int num);
void display_num(int num);
void display_char(int ch3,int ch2,int ch1,int ch0);
//-------------------------------------------------------------------------------------------------------------
void disable_interrupt(void);
void external_Interrupt(void);// INT0 and INT1
void timer0_overflow_Interrupt(void);// timer 0
void timer1_overflow_Interrupt(void);// timer 1
void timer2_overflow_Interrupt(void);// timer 2
void timer1_compare_Match_interrupt(void);// compare 1A and 1B
void timer2_compare_Match_interrupt(void);// compare 2
//-------------------------------------------------------------------------------------------------------------
//MAIM SECTION CODE
int main(void){					
		
		IO_init();		 
		while(1){
						
			sensing();
            thermo_action();
			
			display_char(null,S,n,1);//sensor value 1
			_delay_ms(1000);
			display_num(sens_value_1);// Update the sens_value_0 to display on 7segment
			_delay_ms(1000);
			display_char(null,S,n,2);//sensor value 2
			_delay_ms(1000);
			display_num(sens_value_2);// Update the sens_value_1 to display on 7segment
			_delay_ms(1000);
					
			if(! (button_PIN & 1<<SETUP_SW) ){
				    display_char(null,S,E,t);
				    _delay_ms(100);
					SETUP_flag = 1;
					while(SETUP_flag){
						sensing();
						thermo_setting();
					if(! (button_PIN & 1<<START_SW) ){SETUP_flag = 0;}						  
					}								
				}
			   			   	  	 	 		    
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
	
	output_DDR  |= (1<<PD7) | (1<<PD6) | (1<<PD5) | (1<<PD4);  //Port D[7,6,5,4] as out put
	
	LED_PORT    &=~ (1<<PD4); // LED set off
	BUZ_PORT    &=~ (1<<PD5); // BUZZER set off	
	RL1_PORT    &=~ (1<<PD6); // relay1 set off	
	RL2_PORT    &=~ (1<<PD7); // relay2 set off	
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
		for(channel=0;channel<3;channel++){            //set ADC channel : channel must be 0 to 7  (ADC0....ADC7)
				adc_value   = sensor_read(channel);
				_delay_ms(10);											
				sens_value[channel] = adc_value * 0.46099925;
				ADMUX=0x00;
				_delay_ms(10);								
				}
		sens_value_1  =  (sens_value[0]);
		sens_value_2  =  (sens_value[1]);
		button_value  =  (sens_value[2]);																					   															   												 
}
//-------------------------------------------------------------------------------------------------------------
//Setting Thermo offset //
 void thermo_setting(void){
	 
	 	     display_char(null,P,S,E);
			 _delay_ms(1000);
			 display_num(thermo_set_value1);
			 _delay_ms(1000);
			 display_num(thermo_set_value2);
			 _delay_ms(1000);
			  
			if( button_value == 90 ){ //  thermo_set_SW1 is activated
				       display_char(null,S,E,1);
				       _delay_ms(10);
					   display_char(null,null,U,P);
					   _delay_ms(10);
					   if(thermo_set_value1<1000){ 
						thermo_set_value1 += 1; 
						_delay_ms(10);
						display_num(thermo_set_value1);
								 }//end of if
					   else{thermo_set_value1=0;}
							  }//end of if
			else if(button_value == 207 ){// up_SW is activated
				         display_char(null,S,E,1);
				         _delay_ms(10);
				         display_char(null,d,O,n);
				         _delay_ms(10);
						if(thermo_set_value1>0){
						thermo_set_value1 -= 1;
						_delay_ms(10);
						display_num(thermo_set_value1);
								  }
						else{thermo_set_value1=0;}
							  }//end of if
			else if(button_value == 292){//  down_SW2 is activated
						display_char(null,S,E,2);
						_delay_ms(10);
						display_char(null,null,U,P);
						_delay_ms(10);
						if(thermo_set_value2<1000){
						thermo_set_value2 += 1;
						_delay_ms(10);
						display_num(thermo_set_value2);
								  }//end of if
						 else{thermo_set_value1=0;}
							  }//end of if
			else if(button_value == 392){//  thermo_set_SW2 is activated
						display_char(null,S,E,2);
						_delay_ms(10);
						display_char(null,d,O,n);
						_delay_ms(10);
						if(thermo_set_value2>0){
						thermo_set_value2 -= 1;
						_delay_ms(10);
						display_num(thermo_set_value2);
								 }//end of if
						 else{thermo_set_value2=0;}
							  }				  
}
//-------------------------------------------------------------------------------------------------------------
// Relay 's action by increase or decrease  temperature
void thermo_action(void){
	
	if(sens_value_1 < thermo_set_value1){
		RL1_PORT |= (1<<RL1_BIT) ;//Relay 1 on to increase temperature
	       }//end of if
	else if(sens_value_1 > (thermo_set_value1)){
		RL1_PORT &=~ (1<<RL1_BIT);//Relay 1 off to decrease temperature
	       }//end of if

	if(sens_value_2 < thermo_set_value2){
		RL2_PORT |= (1<<RL2_BIT) ;//Relay 2 on to increase temperature
	       }//end of if
	else if(sens_value_2 > (thermo_set_value2)){
		RL2_PORT &=~ (1<<RL2_BIT);//Relay 2 off to decrease temperature
	       }//end of if
}
//-------------------------------------------------------------------------------------------------------------
//seven segment binary_pattern Hex code//
/*
int code[11]={0xc0,0xf9,0xa4,0xb0,0x99,0x92,0x82,0xf8,0x80,0x90};//
static const uint8_t ascii_table[] = {0xc0, 0xf9, 0xa4, 0xb0, 0x99, 0x92, 0x82, 0xf8, 0x80, 0x90, 0xbf}; // 0 1 2 3 4 5 6 7 8 9 -
static const uint8_t ascii_table[] = {0x3F, 0x6, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x7, 0x7F, 0x6F, 0x40 };	// 0 1 2 3 4 5 6 7 8 9 -

static uint8_t DigitDef[16]={0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7C,0x07,0x7F,0x67,0x77,0x7F,0x39,0x3F,0x79,0x71}; //0 1 2 3 4 5 6 7 8 9 A B C D E F
char hexvalue[] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71};
char ssd[16]={0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F,0x77,0x7C,0x39,0x5E,0x79,0x71};
int segArray[10] = {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f};
unsigned char seg[10]={0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F};
unsigned char arr[10]={0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f};		
//-------------------------------------------------------------------------------------------------------------	
// Code for NPN transistor driver for driving 7-segment
// 0    1    2    3    4    5    6    7    8    9  blank
const char SegCode[11] = {0x40,0x57,0x22,0x06,0x15,0x0C,0x08,0x56,0x00,0x04,0xFF};
const char Column[3] = {0x02,0x01,0x04};	
//-------------------------------------------------------------------------------------------------------------	
unsigned char binary_pattern[]={0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F};
unsigned int a1,a2,a3,a4; // temporary variables to store data of adc
int adc_value; //store output value from Analog Read function
void split_data()
{
	a1 =  adc_value / 1000;   // holds 1000's digit
	a2 = ((adc_value/100)%10); // holds 100's digit
	a3 = ((adc_value/10)%10);  // holds 10th digit
	a4 = (adc_value%10);  // holds unit digit value
}
//-------------------------------------------------------------------------------------------------------------	 
//Connections of the seven segment elements:
//     10 9 7 6 4 2 1 = pin
//   0| -  f a b c d e | 0111111 | 3f
//   1| -  - - b c - - | 0001100 | 0c
//   2| g  - a b - d e | 1011011 | 5b
//   3| g  - a b c d - | 1011110 | 5e
//   4| g  f - b c - - | 1101100 | 6c
//   5| g  f a - c d - | 1110110 | 76
//   6| g  f a - c d e | 1110111 | 77
//   7| -  - a b c - - | 0011100 | 1c
//   8| g  f a b c d e | 1111111 | 7f
//   9| g  f a b c d - | 1111110 | 7e
// digit to seven segment LED mapping:
static unsigned char d2led[]={0x3f,0x0c,0x5b,0x5e,0x6c,0x76,0x77,0x1c,0x7f,0x7e};
//-------------------------------------------------------------------------------------------------------------	
const unsigned char sevensegset[][2] = {
	{' ', 0x00}, {'!', 0x86}, {'"', 0x22}, {'#', 0x7E}, {'$', 0x2D}, {'%', 0xD2}, {'&', 0x7B}, {'(', 0x39}, {')', 0x0F},
	{'*', 0x63}, {'-', 0x40}, {':', 0x09}, {'/', 0x52}, {'<', 0x58}, {'>', 0x4C}, {'=', 0x48}, {'?', 0xD3}, {'@', 0x5F},
	{'0', 0x3F}, {'1', 0x06}, {'2', 0x5B}, {'3', 0x4F}, {'4', 0x66}, {'5', 0x6D}, {'6', 0x7D}, {'7', 0x07}, {'8', 0x7F},
	{'9', 0x6F}, {'A', 0x77}, {'B', 0x7C}, {'C', 0x39}, {'D', 0x5E}, {'E', 0x79}, {'F', 0x71}, {'G', 0x3D}, {'H', 0x76},
	{'I', 0x30}, {'J', 0x1E}, {'K', 0x75}, {'L', 0x38}, {'M', 0x37}, {'N', 0x54}, {'O', 0x3F}, {'P', 0x73}, {'Q', 0x67},
	{'R', 0x50}, {'S', 0x6D}, {'T', 0x78}, {'U', 0x3E}, {'V', 0x1C}, {'W', 0x2A}, {'X', 0x76}, {'Y', 0x6E}, {'Z', 0x5B},
	{'a', 0x77}, {'b', 0x7C}, {'c', 0x58}, {'d', 0x5E}, {'e', 0x79}, {'f', 0x71}, {'g', 0x3D}, {'h', 0x74}, {'i', 0x10},
	{'j', 0x1E}, {'k', 0x75}, {'l', 0x38}, {'m', 0x37}, {'n', 0x54}, {'o', 0x5C}, {'p', 0x73}, {'q', 0x67}, {'r', 0x50},
	{'s', 0x6D}, {'t', 0x78}, {'u', 0x3E}, {'v', 0x1C}, {'w', 0x2A}, {'x', 0x76}, {'y', 0x6E}, {'z', 0x5B}, {'.', 0x80}
};		 		
//-------------------------------------------------------------------------------------------------------------				
//------ Function to Return mask for common anode 7 segment display.
unsigned short mask(unsigned short num) {
	switch (num) {
		case 0 : return 0xC0;
		case 1 : return 0xF9;
		case 2 : return 0xA4;
		case 3 : return 0xB0;
		case 4 : return 0x99;
		case 5 : return 0x92;
		case 6 : return 0x82;
		case 7 : return 0xF8;
		case 8 : return 0x80;
		case 9 : return 0x90;
	} //case end
}
//-------------------------------------------------------------------------------------------------------------		
*/
//-------------------------------------------------------------------------------------------------------------
//segment Hex code//
int segment_Code(int num){ //code for number to setting segment_PORT of MCU

		 switch (num){
			           //  . g f e d c b a
			 case 0:   return 0b00111111;break;//0X3F
			 case 1:   return 0b00000110;break;//0X06
			 case 2:   return 0b01011011;break;//0X5B
			 case 3:   return 0b01001111;break;//0X4F
			 case 4:   return 0b01100110;break;//0X66
			 case 5:   return 0b01101101;break;//0X6D
			 case 6:   return 0b01111101;break;//0X7D
			 case 7:   return 0b00000111;break;//0X07
			 case 8:   return 0b01111111;break;//0X7F
			 case 9:   return 0b01101111;break;//0X6F//
			 case 20:  return 0b01110111;break;//0X77//A
			 case 30:  return 0b01111100;break;//0X7C//b
			 case 40:  return 0b00111001;break;//0X39//C
			 case 50:  return 0b01011110;break;//0X5E//d
			 case 60:  return 0b01111001;break;//0X79//E
			 case 70:  return 0b01110001;break;//0X71//F
			 case 80:  return 0b01110110;break;//0X76//H
			 case 90:  return 0b00111111;break;//0X3F//O
			 case 100: return 0b00111000;break;//0X38//L
			 case 110: return 0b00110111;break;//0X37//n
			 case 120: return 0b00111110;break;//0X3E//U
			 case 130: return 0b01101101;break;//0X3E//S
			 case 140: return 0b01111000;break;//0X3E//t
			 case 150: return 0b01110011;break;//0X3E//P			 
			 case 200: return 0b10000000;break;//0X80 //if decimal point should be displayed make 7th bit high
			 case 210: return 0b00000000;break;//0X00 // clear or Null
		               }
 }
//-------------------------------------------------------------------------------------------------------------
//display_number//
void display_num(int num){
		
	segment_buffer[3] =   num / 1000;// holds 1000's digit
	segment_buffer[2] = ((num/100)%10);// holds 100's digit
	segment_buffer[1] = ((num/10)%10);// holds 10th digit
	segment_buffer[0] =  (num%10);
	   
	for(int i=0;i<50;i++){
		
		digit_PORT   |= (1<<PD2) | (1<<PD1) | (1<<PD0);
		digit_PORT   &=~(1<<PD3);
		segment_PORT =  segment_Code(segment_buffer[3]);
		_delay_ms(5);
		
		digit_PORT   |= (1<<PD3) | (2<<PD1) | (1<<PD0);
		digit_PORT   &=~(1<<PD2);
		segment_PORT =  segment_Code(segment_buffer[2]);
		_delay_ms(5);
		
		digit_PORT   |= (1<<PD3) | (1<<PD2) | (1<<PD0);
		digit_PORT   &=~(1<<PD1);
		segment_PORT =  segment_Code(segment_buffer[1]);
		_delay_ms(5);
		
		digit_PORT   |= (1<<PD3) | (1<<PD2) | (1<<PD1);
		digit_PORT   &=~(1<<PD0);
		segment_PORT =  segment_Code(segment_buffer[0]);
		_delay_ms(5);
	}
}
//-------------------------------------------------------------------------------------------------------------
//display_chracter//
void display_char(int ch3,int ch2,int ch1,int ch0){	
	
	char_buffer[2] =   ch3;    			   
	char_buffer[2] =   ch2;   
	char_buffer[1] =   ch1; 
	char_buffer[0] =   ch0;
	  
	for(int i=0;i<50;i++){
		
		digit_PORT   |= (1<<PD2) | (2<<PD1) | (1<<PD0);
		digit_PORT   &=~(1<<PD3);
		segment_PORT =  segment_Code(char_buffer[2]);
		_delay_ms(5);
		
		digit_PORT   |= (1<<PD3) | (2<<PD1) | (1<<PD0);
		digit_PORT   &=~(1<<PD2);
		segment_PORT =  segment_Code(char_buffer[2]);
		_delay_ms(5);
		
		digit_PORT   |= (1<<PD3) | (1<<PD2) | (1<<PD0);
		digit_PORT   &=~(1<<PD1);
		segment_PORT =  segment_Code(char_buffer[1]);
		_delay_ms(5);
		
		digit_PORT   |= (1<<PD3) | (1<<PD2) | (1<<PD1);
		digit_PORT   &=~(1<<PD0);
		segment_PORT =  segment_Code(char_buffer[0]);
		_delay_ms(5);
	}		   
}
//-------------------------------------------------------------------------------------------------------------

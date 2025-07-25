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
//-------------------------------------------------------------------------------------------------------------
#define segment_PORT   PORTB
#define segment_DDR    DDRB

#define digit_PORT     PORTD
#define digit_DDR      DDRD

#define sensor_PORT    PORTC
#define sensor_DDR     DDRC
#define sensor_PIN     PINC
//-------------------------------------------------------------------------------------------------------------
#define BUTTON_HOLD_MS   5000   // Press button for 3 second
//-------------------------------------------------------------------------------------------------------------
#define BUTTON_PORT PORTC           // PORTx - register for button output 
#define BUTTON_PIN  PINC            // PINx -  register for button input 
#define BUTTON_DDR  DDRC
#define thermo_set_SW1         PC2
#define thermo_set_SW2         PC3
#define up_SW                  PC4
#define down_SW                PC5
#define POWER_SW               PC2
//#define exINT0_SW             PD2
//#define exINT1_SW             PD3
//-------------------------------------------------------------------------------------------------------------
#define LED_DDR  DDRD               // LED data direction register 
#define LED_PORT PORTD              // PORTx - register for LED output 
#define LED_BIT  PD4                // bit for LED output 

#define BUZ_DDR  DDRD               // LED data direction register
#define BUZ_PORT PORTD              // PORTx - register for LED output
#define BUZ_BIT  PD5                // bit for LED output

#define RL1_DDR  DDRD               // RL1 data direction register
#define RL1_PORT PORTD              // PORTx - register for RL1 output
#define RL1_BIT  PD6                // bit for RL1 output 

#define RL2_DDR  DDRD               // RL2 data direction register
#define RL2_PORT PORTD              // PORTx - register for RL2 output
#define RL2_BIT  PD7                // bit for RL2 output
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
#define u     120 // 0b00111110
#define dot   200 // 0b10000000
#define null  210 // 0b00000000
//-------------------------------------------------------------------------------------------------------------
void IO_init(void);
void disable_interrupt(void);
void external_Interrupt(void);// INT0 and INT1
void timer0_overflow_Interrupt(void);// timer 0 
void timer1_overflow_Interrupt(void);// timer 1 
void timer2_overflow_Interrupt(void);// timer 2
void timer1_compare_Match_interrupt(void);// compare 1A and 1B
void timer2_compare_Match_interrupt(void);// compare 2
//-------------------------------------------------------------------------------------------------------------
void POWER_OFF(void);
void main_program();
//-------------------------------------------------------------------------------------------------------------
int  segment_Code(int data);
void display_num(int number);
void display_char(int code , int char_pos);
//-------------------------------------------------------------------------------------------------------------             
unsigned char           segment_buffer[4]; // 4 digit  7segment display
static unsigned char    dot_pos = 3;
//-------------------------------------------------------------------------------------------------------------
int thermo_set_SW1_flag;
int thermo_set_SW2_flag;
int up_SW_flag;
int down_SW_flag;
int thermo_set_value1; 
int thermo_set_value2;
int POWER_SW_FLAG;
int exINT0_SW_flag;
int exINT1_SW_flag; 
//------------------------------------------------------------------------------------------------------------- 
//MAIM SECTION CODE
int main(void){					
		
		IO_init();
		//external_Interrupt(); 
		
		timer0_overflow_Interrupt();
		//timer1_overflow_Interrupt();
	    //timer2_overflow_Interrupt();
		//timer1_compare_Match_interrupt();
		//timer2_compare_Match_interrupt();
				
		//sei();//   enable interrupt globally

				while(1){	
					display_num(350);								            					   						  						 						   					     
							
							
							
							while(! (BUTTON_PIN  &  1<<POWER_SW ) ){																	
									POWER_SW_FLAG ++;// the POWER_SW is pressed more thane 1 second
									_delay_ms(1);
									if(POWER_SW_FLAG > BUTTON_HOLD_MS){// Check if button has been pressed enough
										segment_PORT =0b10000000;
											   POWER_OFF();
									          }//end of if
							        else{								       
									        // main code here //
											display_num(350);
											main_program();
								                  }//end of else
							       }//end of power_off while	   								   
								   			   	  	 	 		    
				  }//end of main while		                     	     						         
    return 0;
}//end of main
//-------------------------------------------------------------------------------------------------------------
//ALL interrupt disable
void disable_interrupt(void){
	    //cli();
		GICR   &=~ (1<<INT0);//  Enable  INT0 
		GICR   &=~ (1<<INT1);//  Enable  INT1
		TIMSK  &=~ (1<<TOIE0);// Enable Timer 0 overflow interrupt
	    TIMSK  &=~ (1<<TOIE1);// Enable Timer 1 overflow interrupt
	    TIMSK  &=~ (1<<TOIE2);// Enable Timer 2 overflow interrupt
		TIMSK  &=~ (1 << OCIE1A);// Output Compare A Match Interrupt Enable
		TIMSK  &=~ (1 << OCIE1B);// Output Compare B Match Interrupt Enable
		TIMSK  &=~ (1 << OCIE2);//  Output Compare  Match Interrupt Enable.page 120 of ATmega8 data sheet.			
}
//-------------------------------------------------------------------------------------------------------------
//IO INITIALISE
void IO_init(void){
	
	segment_DDR = 0XFF; //Port B [7,6,5,3,2,1,0] as [dot,G,F,E,D,C,B,A]
	digit_DDR   = 0XFF;  //Port D[5,4,1,0] as out put
	
	sensor_DDR  &= ~(1<<PC0); // sensor1  termo_readValue_1
	sensor_DDR  &= ~(1<<PC1); // sensor2  termo_readValue_2
	sensor_PORT |=  (1<<PC0); //enable internal pull-up resistors in PC0
	sensor_PORT |=  (1<<PC1); //enable internal pull-up resistors in PC1
	
	BUTTON_DDR  &= ~(1<<PC2); // sw1  termo_setValue_1
	BUTTON_DDR  &= ~(1<<PC3); // sw2  termo_setValue_2
	BUTTON_DDR  &= ~(1<<PC4); // sw3  up   increment
	BUTTON_DDR  &= ~(1<<PC5); // sw4  down decrement
	
	BUTTON_PORT |=  (1<<PC2); //enable internal pull-up resistors in PC2
	BUTTON_PORT |=  (1<<PC3); //enable internal pull-up resistors in PC3
	BUTTON_PORT |=  (1<<PC4); //enable internal pull-up resistors in PC4
	BUTTON_PORT |=  (1<<PC5); //enable internal pull-up resistors in PC5
	
	LED_DDR     |= (1<<PD4); // PD4 as out put
	LED_PORT    &=~(1<<PD4); // LED set off
	BUZ_DDR     |= (1<<PD5); // PD5 as out put
	BUZ_PORT    &=~(1<<PD5); // BUZZER set off	
	RL1_DDR     |= (1<<PD6); // PD6 as out put
	RL1_PORT    &=~(1<<PD6); // relay1 set off	
	RL2_DDR     |= (1<<PD7); // PD7 as out put
	RL2_PORT    &=~(1<<PD7); // relay2 set off
	
}
//-------------------------------------------------------------------------------------------------------------
void main_program(){
	
		if(! (BUTTON_PIN  &  1<<thermo_set_SW1 ) ){ // the termo_set_SW1 is low?
			thermo_set_SW1_flag = 1;
			thermo_set_SW2_flag = 0;
				
				while(thermo_set_SW1_flag==1){
					if(! (BUTTON_PIN  &  1<<up_SW )){
						thermo_set_value1 +=10;
						display_char(b,1);
						display_num(thermo_set_value1);
						}//end of if
					 if(! (BUTTON_PIN  &  1<<down_SW )){
						thermo_set_value1 -=10;
						display_char(u,2);
						display_num(thermo_set_value1);
						}//end of if
					 if(! (BUTTON_PIN  &  1<<thermo_set_SW2 ) ){
						 thermo_set_SW1_flag==0;
						 break;
						 }							
				  }//end of while
			}//end of if

		 if(! (BUTTON_PIN  &  1<<thermo_set_SW2 ) ){ // the termo_set_SW2 is low?
				thermo_set_SW1_flag = 0;
				thermo_set_SW2_flag = 1;
	
				while(thermo_set_SW2_flag==1){
					if(! (BUTTON_PIN  &  1<<up_SW )){
						    thermo_set_value2 +=10;
						    display_char(F,1);
						    display_num(thermo_set_value2);
						    }//end of if 
				    if(! (BUTTON_PIN  &  1<<down_SW )){
						    thermo_set_value2 -=10;
						    display_char(E,2);
						    display_num(thermo_set_value2);
						    }//end of if 
					  if(! (BUTTON_PIN  &  1<<thermo_set_SW1 ) ){
						  thermo_set_SW2_flag==0;
						  break;
					  }	 
			        }//end of while
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
int segment_Code(int data){ //code for number to setting segment_PORT of MCU

		 switch (data){
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
			 case 120: return 0b00111110;break;//0X3E//u			 
			 case 200: return 0b10000000;break;//0X80 //if decimal point should be displayed make 7th bit high
			 case 210: return 0b00000000;break;//0X00 // clear or Null
		               }
 }
//-------------------------------------------------------------------------------------------------------------
//display_number//
void display_num(int number){
		
	segment_buffer[3] =   number / 1000;    // holds 1000's digit
	segment_buffer[2] = ((number/100)%10); // holds 100's digit
	segment_buffer[1] = ((number/10)%10);  // holds 10th digit
	segment_buffer[0] =  (number%10);
}
//-------------------------------------------------------------------------------------------------------------
//display_chracter
void display_char(int code , int char_pos){
	    
  segment_PORT = segment_Code(code);
  //Port D[5,4,1,0] as out put 0b00110011  (pd5,pd4,pd1,pd0)
  //Port D[3,2,1,0] as out put 0b00001111  (pd3,pd2,pd1,pd0)
			if(char_pos == 0){  		  
				digit_PORT   = 0b00001110 ;//0b00110010
				   }
			else if(char_pos == 1){
				digit_PORT   = 0b00001101 ;//0b00110001
				   }
			else if(char_pos == 2){
				digit_PORT   = 0b00001011 ;//0b00100011
				   }
			else if(char_pos == 3){
				digit_PORT   = 0b00000111 ;//0b00010011
				   }
	}
//-------------------------------------------------------------------------------------------------------------
//MCU Control Register
/*
MCU Control Register –MCUCR

Bit 7 6 5 4 3 2 1 0
SE SM2 SM1 SM0 ISC11 ISC10 ISC01 ISC00 MCUCR
SE: Sleep Enable

Table 13. Sleep Mode Select  page 13
SM2 SM1 SM0 Sleep Mode
0 0 0       Idle
0 0 1       ADC Noise Reduction
0 1 0       Power-down
0 1 1       Power-save
1 0 0       Reserved
1 0 1       Reserved
1 1 0       Standby(1)
Note: 1. Standby mode is only available with external crystals or resonators.

Watchdog Timer Control Register – WDTCR

Bit 7 6 5 4 3 2 1 0
– – – WDCE WDE WDP2 WDP1 WDP0 WDTCR

WDCE: Watchdog Change Enable
WDE: Watchdog Enable
WDP2, WDP1, WDP0: Watchdog Timer Prescaler 2, 1, and 0 page 42
*/
//Timer/Counter prescaler
//-------------------------------------------------------------------------------------------------------------	
/*	
//Timer/Counter Control Register – TCCR0
CS02 CS01 CS00	
//001         clkI/O/(No prescaling)
//010         clkI/O/8 (From prescaler)
//011         clkI/O/64 (From prescaler)
//100         clkI/O/256 (From prescaler)
//101         clkI/O/1024 (From prescaler)
	
//Timer/Counter Control Register – TCCR1B
CS12 CS11 CS10
//001         clkI/O/1 (No prescaling)
//010         clkI/O/8 (From prescaler)
//011         clkI/O/64 (From prescaler)
//100         clkI/O/256 (From prescaler)
//101         clkI/O/1024 (From prescaler)

//Timer/Counter Control Register – TCCR2
CS22 CS21 CS20
//001           clkT2S/(No prescaling)
//010           clkT2S/8 (From prescaler)
//011           clkT2S/32 (From prescaler)
//100           clkT2S/64 (From prescaler)
//101           clkT2S/128 (From prescaler)
//110           clkT2S/256 (From prescaler)
//111           clkT2S/1024 (From prescaler)

// 8-bit timer0 ISR
   * 1/8MHz * (2^8) =    32us
   * 8/8MHz * (2^8) =    256us
   * 64/8MHz * (2^8) =   2.048ms
   * 256/8MHz * (2^8) =  8.192ms
   * 1024/8MHz * (2^8) = 32.768ms
   
// 16-bit timer1 ISR
	* Enable to increment count on timer overflow @ 2^16 (16-bit timer)
	* Timer Resolution = prescaler/input frequency
	* 1/8MHz * (2^16) =    8.192ms
	* 8/8MHz * (2^16) =    65.536ms
	* 64/8MHz * (2^16) =   524.288ms
	* 256/8MHz * (2^16) =  2.097s
	* 1024/8MHz * (2^16) = 8.389s   
	
// 8-bit timer2 ISR
	* 1/8MHz * (2^8) =    32us
	* 8/8MHz * (2^8) =    256us
	* 32/8MHz * (2^8) =   1024us
	* 64/8MHz * (2^8) =   2.048ms
	* 128/8MHz * (2^8) =  4.096ms
	* 256/8MHz * (2^8) =  8.192ms
	* 1024/8MHz * (2^8) = 32.768ms	
*/
//-------------------------------------------------------------------------------------------------------------
//WDT_off//
/*
void WDT_off(void){
	
	// reset WDT 
	// Write logical one to WDCE and WDE
	WDTCR |= (1<<WDCE) | (1<<WDE);
	// Turn off WDT
	WDTCR = 0x00;
}
*/
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
//EXTERNAL INTERRUPT
void external_Interrupt(void){

	GICR  |=(1<<INT0);// Enable INT0
	GICR  |=(1<<INT1);// Enable INT1
	MCUCR |= (1 << ISC00);// INT0 is executed on every edge
	MCUCR |= (1 << ISC10);// INT1 is executed on every edge
	DDRD  &= ~(1 << PD2);// Clear the PD2 pin  PD2 (INT0 pin) is now an input
	DDRD  &= ~(1 << PD3);// Clear the PD3 pin  PD3 (INT1 pin) is now an input
	sei();	
}
//-------------------------------------------------------------------------------------------------------------
//Interrupt Service Routine (ISR) for INT0
ISR(INT0_vect){	
	RL1_PORT |=  (1<<RL1_BIT);
	RL2_PORT &=~ (1<<RL2_BIT);	
}
//-------------------------------------------------------------------------------------------------------------
//Interrupt Service Routine (ISR) for INT1
ISR(INT1_vect){
	RL1_PORT &=~ (1<<RL1_BIT);
	RL2_PORT |=  (1<<RL2_BIT);		
}
//-------------------------------------------------------------------------------------------------------------
// Timer0 Overflow Interrupt 
// refresh rate of display
void timer0_overflow_Interrupt(void){
	
	TCNT0 = 0;// load TCNT0, count for 10ms.if TCNT0 = 0xC2 ==>> 0b11000010 =128+64+2=194		
	TCCR0   |= (1 << CS01);// Start Timer0 with prescalar = 1
	//(8MHz clock gives overflow interrupts every  1/8MHz * (2^8) = 32us)
	TIMSK=(1<<TOIE0);// Enable Timer0 overflow interrupts
	sei(); 	
}
//-------------------------------------------------------------------------------------------------------------
// Timer0 Overflow vector subroutine 
ISR(TIMER0_OVF_vect){
	 
	static unsigned char  digit_pos;
		//Port D[5,4,1,0] as out put 0b00110011  (pd5,pd4,pd1,pd0)
	segment_PORT = 0x00 ;  //digit_NULL
	if(digit_pos == 0){
		digit_PORT   = 0b00001110 ;
	}
	else if(digit_pos == 1){
		digit_PORT   = 0b00001101 ;
	}
	else if(digit_pos == 2){
		digit_PORT   = 0b00001011;
	}
	else if(digit_pos == 3){
		digit_PORT   = 0b00000111 ;
	}
	segment_PORT =  segment_Code(segment_buffer[digit_pos]);   // Common Cathode display (high bit on segments and low bit on digits)	ABCDEFG
	if(dot_pos == digit_pos) {
		segment_PORT |= segment_Code(80);   // If decimal point need to be displayed
	}
	digit_pos++;
	if(digit_pos > 3){digit_pos = 0;}   			  			 			 
}
//-------------------------------------------------------------------------------------------------------------
// Timer1 Overflow Interrupt 
void timer1_overflow_Interrupt(void){
	TCNT1 = 0;// 2^16 /2 = 65536/2 = 32768
	TCCR1B |= (1 << CS11);// Start Timer1 with prescalar = 8 
	//(8MHz clock gives overflow interrupts every  8/8MHz * (2^16) = 65.536ms
	TIMSK  |= (1<<TOIE1);// Enable Timer 1 overflow interrupt
	sei();		
}
//-------------------------------------------------------------------------------------------------------------
// Timer1 Overflow vector // 
ISR(TIMER1_OVF_vect){	
  segment_PORT ^= 0b01111001;  
}
//-------------------------------------------------------------------------------------------------------------
// Timer2 Overflow Interrupt  
void timer2_overflow_Interrupt(void){	
	TCNT2 =0;
	TCCR2  |= (1 << CS21);// Start Timer2 with prescalar = 8 
	//(8MHz clock gives overflow interrupts every 8/8MHz * (2^8) = 256us)
	TIMSK  |= (1 << TOIE2);// Enable Timer 2 overflow interrupt	
	sei();	
}
//-------------------------------------------------------------------------------------------------------------
// Timer2 Overflow vector //
ISR(TIMER2_OVF_vect){

	segment_PORT ^= 0b10000000;
}
//-------------------------------------------------------------------------------------------------------------
// Timer1 COMPARE MATCH INTERRUPT//
void timer1_compare_Match_interrupt(void){
	
		//set timer1 interrupt at 1Hz
		TCCR1A = 0;// set entire TCCR1A register to 0
		TCCR1B = 0;// same for TCCR1B
		TCNT1  = 0;//initialize counter1 value to 0	
		//TCCR1A |= (1 << WGM12);// CTC mode
		//TCCR1A |= (1 << COM1A0);// CTC mode, toggle OC1A on compare match
		//TCCR1B |= (1 << COM1B0);// Toggles OC1B pin each cycle through
        TCCR1B |= (1 << CS11);// set up timer with prescaler = clock/8 	
		TIMSK |= (1 << OCIE1A);// Output Compare A Match Interrupt Enable
		TIMSK |= (1 << OCIE1B);// Output Compare B Match Interrupt Enable
		// set compare match register for 1hz increments. = (16*10^6) / (1*1024) - 1 (must be <65536)
		//OCR1A = 15625;// initialize compare value.around 1000ms delay
		OCR1A = 3906;
		OCR1B = 7812;// initialize compare value.around 500ms delay
		//OCR1B = 125;//initialize compare value.around 1ms delay	
		 
		// F_Timer = F_clk/Prescaler
		// = 8'000'000 / 1024 = 7812.5
		// F_PDx = F_Timer/ 2 / OCR1A
		// = 7812.5 / 2 / 3906 = 1.00006Hz
		sei();
}
//-------------------------------------------------------------------------------------------------------------
// Timer1 Compare Match A vector //
ISR(TIMER1_COMPA_vect) {	
	LED_PORT |= 1<< LED_BIT;		
}
//-------------------------------------------------------------------------------------------------------------
// Timer1 Compare Match B vector //
ISR(TIMER1_COMPB_vect) {
	BUZ_PORT |= 1<<BUZ_BIT;
}
//-------------------------------------------------------------------------------------------------------------
// Timer2 COMPARE MATCH INTERRUPT//
void timer2_compare_Match_interrupt(void){
	
			//set timer2 interrupt at 1Hz
			TCCR2 = 0;// set entire TCCR2 register to 0
			TCNT2 = 0;//initialize counter2 value to 0
			//TCCR2 |= (1 << WGM21);// CTC mode
			//TCCR2 |= (1 << COM21)|(1 << COM20);// CTC mode, Set OC2 on Compare Match . page 116 of ATmega8 data sheet.
			TCCR2 |= (1 << CS22)|(1 << CS21)|(1 << CS20);// set up timer with prescaler. page 117 of ATmega8 data sheet. F_Timer = F_clock/Prescaler.
			TIMSK |= (1 << OCIE2);// Output Compare  Match Interrupt Enable.page 120 of ATmega8 data sheet.
			
			// set compare match register for delay.  most be < (F_clock/Prescaler - 1)
			//OCR2 = 7812;//  initialize compare value. delay = F_Timer /2 /OCR2 
			//OCR2 = 125;// initialize compare value. delay = F_Timer /2 /OCR2 						
			//if OCR2 = 3906 , clock = 8MHz , prescaler = 1024 ===>>
			// = 8'000'000 /1024 = 7812.5
			// = 7812.5 /2 /3906 = 1.00006Hz
			sei();				
}
//-------------------------------------------------------------------------------------------------------------
// Timer2 Compare Match vector //
ISR(TIMER2_COMP_vect){
	
	LED_PORT &=~ 1<<LED_BIT;
	BUZ_PORT &=~ 1<<BUZ_BIT;		
}
//-------------------------------------------------------------------------------------------------------------
//Question: When OC1A toggle?
//Answer: when TCNT1 == OCR1A
//Question: when OC1B toggle?
//Answer: when TCNT1 == OCR1B
//-------------------------------------------------------------------------------------------------------------

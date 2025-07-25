/*
 * display.h
 *
 * Created: 2/2/2024 17:04:43
 *  Author: me
 */ 
//-------------------------------------------------------------------------------------------------------------
#ifndef DISPLAY_H_
#define DISPLAY_H_
//-------------------------------------------------------------------------------------------------------------
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
//-------------------------------------------------------------------------------------------------------------
#define a     20  // 0b01110111
#define b     30  // 0b01111100
#define c     40  // 0b00111001
#define d     50  // 0b01011110
#define e     60  // 0b01111001
#define f     70  // 0b01110001
#define h     80  // 0b01110110
#define o     90  // 0b00111111
#define l     100 // 0b00111000
#define n     110 // 0b00110111
#define u     120 // 0b00111110
#define s     130 // 0b01101101
#define t     140 // 0b01111000
#define p     150 // 0b01110011
#define dot   200 // 0b10000000
#define null  210 // 0b00000000
//-------------------------------------------------------------------------------------------------------------
#define segment_PORT   PORTB
#define segment_DDR    DDRB
#define digit_PORT     PORTD
#define digit_DDR      DDRD
//-------------------------------------------------------------------------------------------------------------
uint16_t    dot_pos = 3;
uint16_t    ch3,ch2,ch1,ch0;
//-------------------------------------------------------------------------------------------------------------
uint16_t  segment_Code(uint16_t num);
uint16_t  spelit_num(uint16_t num);
void display_num(uint16_t num,uint16_t cycle);
void display_char(uint16_t ch3,uint16_t ch2,uint16_t ch1,uint16_t ch0);
void moving_display(uint16_t ch3,uint16_t ch2,uint16_t ch1,uint16_t ch0);
void message( uint16_t T_sens,uint16_t num,uint16_t RL_status);
//-------------------------------------------------------------------------------------------------------------
//segment Hex code//
uint16_t segment_Code(uint16_t num){ //code for number to setting segment_PORT of MCU

		 switch (num){
			           //  . g f e d c b a
			 case 0:   return 0b00111111;break;//0X3F
			 case 1:   return 0b00110000;break;//0X06
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
uint16_t spelit_num(uint16_t num){
		/*
		segment_buffer[3] = segment_Code( num / 1000);// holds 1000's digit
		segment_buffer[2] = segment_Code(((num/100)%10));// holds 100's digit
		segment_buffer[1] = segment_Code(((num/10)%10));// holds 10th digit
		segment_buffer[0] = segment_Code((num%10));
		*/
		uint16_t digit;
		uint16_t   segment_buffer[4]; // 4 digit  7segment display
		
		digit = num/1000;
		segment_buffer[3] = segment_Code(digit);
		num -= (digit*1000);
		
		digit = num/100;
		segment_buffer[2] = segment_Code(digit);
		num -= (digit*100);
		
		digit = num/10;
		segment_buffer[1] = segment_Code(digit);
		num -= (digit*10);
		
		digit = num;
		segment_buffer[0] = segment_Code(digit);
		
		ch3 = segment_buffer[3];
		ch2 = segment_buffer[2];
		ch1 = segment_buffer[1];
		ch0 = segment_buffer[0];
		
		return ch3,ch2,ch1,ch0;
}
//-------------------------------------------------------------------------------------------------------------
//display_number//
void display_num(uint16_t num,uint16_t cycle){
	
	spelit_num(num); 
	for(int i=0;i<cycle;i++){
		
		digit_PORT   |= (1<<PD2) | (1<<PD1) | (1<<PD0);
		digit_PORT   &=~(1<<PD3);
		segment_PORT =  segment_Code(ch3);
		_delay_ms(5);
		
		digit_PORT   |= (1<<PD3) | (1<<PD1) | (1<<PD0);
		digit_PORT   &=~(1<<PD2);
		segment_PORT =  segment_Code(ch2);
		_delay_ms(5);
		
		digit_PORT   |= (1<<PD3) | (1<<PD2) | (1<<PD0);
		digit_PORT   &=~(1<<PD1);
		segment_PORT =  segment_Code(ch1);
		_delay_ms(5);
		
		digit_PORT   |= (1<<PD3) | (1<<PD2) | (1<<PD1);
		digit_PORT   &=~(1<<PD0);
		segment_PORT =  segment_Code(ch0);
		_delay_ms(5);
		
		segment_PORT = segment_Code(null);
	    }
		
}
//-------------------------------------------------------------------------------------------------------------
//display_chracter//
void display_char(uint16_t ch3,uint16_t ch2,uint16_t ch1,uint16_t ch0){		  
	for(int i=0;i<50;i++){
		
		digit_PORT   |= (1<<PD2) | (2<<PD1) | (1<<PD0);
		digit_PORT   &=~(1<<PD3);
		segment_PORT =  segment_Code(ch3);
		_delay_ms(5);
		
		digit_PORT   |= (1<<PD3) | (2<<PD1) | (1<<PD0);
		digit_PORT   &=~(1<<PD2);
		segment_PORT =  segment_Code(ch2);
		_delay_ms(5);
		
		digit_PORT   |= (1<<PD3) | (1<<PD2) | (1<<PD0);
		digit_PORT   &=~(1<<PD1);
		segment_PORT =  segment_Code(ch1);
		_delay_ms(5);
		
		digit_PORT   |= (1<<PD3) | (1<<PD2) | (1<<PD1);
		digit_PORT   &=~(1<<PD0);
		segment_PORT =  segment_Code(ch0);
		_delay_ms(5);
		
		segment_PORT = segment_Code(null);
	    }		   
}
//-------------------------------------------------------------------------------------------------------------
void moving_display(uint16_t ch3,uint16_t ch2,uint16_t ch1,uint16_t ch0){
	
		display_char(null,null,null,ch3);
		display_char(null,null,ch3,ch2);
		display_char(null,ch3,ch2,ch1);
		display_char(ch3,ch2,ch1,ch0);
		display_char(ch2,ch1,ch0,null);
		display_char(ch1,ch0,null,null);
		display_char(ch0,null,null,null);
		display_char(null,null,null,null);	
}
//-------------------------------------------------------------------------------------------------------------
void message( uint16_t T_sens,uint16_t num,uint16_t RL_status){
	moving_display(null,null,c,num);//sensor value 
	//spelit_num(T_sens);
	//moving_display(ch3,ch2,ch1,ch0);
	//moving_display(null,L,num,state);
	//display_num(T_sens);
}
//-------------------------------------------------------------------------------------------------------------
#endif /* DISPLAY_H_ */
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
//program for splitting a string//
/*
void string(){
	
char str1[] = "T1 main heat";
char str2[] = "T2 preheat";
int init_size_1 = strlen(str1);
int init_size_2 = strlen(str2);

		for (int i = 0; i < init_size_1; i++){	
			display_char(null,null,null,str1[i]);
			display_char(null,null,str1[i],str1[i+1]);
			display_char(null,str1[i],str1[i+1],str1[i+2]);
			display_char(str1[i],str1[i+1],str1[i+2],str1[i+3]);
			display_char(str1[i+1],str1[i+2],str1[i+3],null);
			display_char(str1[i+2],str1[i+3],null,null);
			display_char(str1[i+3],null,null,null);
			display_char(null,null,null,null);
			}
			
			// Splits str[] according to given delimiters.
			// and returns next token. It needs to be called
			// in a loop to get all tokens. It returns NULL
			// when there are no more tokens.
			//char * strtok(char str[], const char *delims);
			// A C/C++ program for splitting a string
			// using strtok()
			#include <stdio.h>
			#include <string.h>
			
			int splitting()
			{
				char str[] = "Geeks-for-Geeks";
				
				// Returns first token
				char *token = strtok(str, "-");
				
				// Keep printing tokens while one of the
				// delimiters present in str[].
				while (token != NULL)
				{
					printf("%s\n", token);
					token = strtok(NULL, "-");
				}
				
				return 0;
			}
			
			
			#include <stdio.h>
			#include <string.h>

			int main() {
				char str1[100];         // String input
				char newString[10][10]; // 2D array to store split strings
				int i, j, ctr;          // Counters and index variables

				printf("\n\n Split string by space into words :\n");
				printf("---------------------------------------\n");

				printf(" Input a string : ");
				fgets(str1, sizeof str1, stdin); // Read input string

				j = 0;    // Initialize column index of newString array
				ctr = 0;  // Initialize word count

				// Loop through each character in the input string
				for (i = 0; i <= (strlen(str1)); i++) {
					// If space or NULL found, assign NULL into newString[ctr]
					if (str1[i] == ' ' || str1[i] == '\0') {
						newString[ctr][j] = '\0'; // Null-terminate the word
						ctr++;  // Move to the next word
						j = 0;  // Reset column index to 0 for the next word
						} else {
						newString[ctr][j] = str1[i]; // Store the character into newString
						j++;  // Move to the next character within the word
					}
				}

				printf("\n Strings or words after split by space are :\n");
				// Display the split words stored in newString array
				for (i = 0; i < ctr; i++) {
					printf(" %s\n", newString[i]);
				}

				return 0;
			}	
}
*/

//-------------------------------------------------------------------------------------------------------------
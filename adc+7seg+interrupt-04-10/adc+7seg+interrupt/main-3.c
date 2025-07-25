/*
 * adc+7seg+interrupt.c
 * ver.4.03
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
#define BUTTON_HOLD_MS   3000   // Press button for 1 second
//-------------------------------------------------------------------------------------------------------------
#define BUTTON_PORT PORTC           // PORTx - register for button output 
#define BUTTON_PIN  PINC            // PINx - register for button input 
#define BUTTON_DDR  DDRC
#define SW1         PC2
#define SW2         PC3
#define SW3         PC4
#define SW4         PC5
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
void IO_init(void);
void external_Interrupt(void);//for INT0 and INT1
void overflow_Interrupt(void);//for timer 0 and 1 and 2
void Timer1_compare_Match_interrupt(void);//for compare 1A and 1B
void Timer2_compare_Match_interrupt(void);//for compare 2
//-------------------------------------------------------------------------------------------------------------
void POWER_OFF(void);
//-------------------------------------------------------------------------------------------------------------
int flag_1;
int flag_2;
int flag_3;
int POWER_FLAG=0;  
//------------------------------------------------------------------------------------------------------------- 
//MAIM SECTION CODE
int main(void){					
		
		IO_init();
		
		external_Interrupt(); 
		Timer1_compare_Match_interrupt();
		overflow_Interrupt();
		sei();// Set the I-bit in SREG  enable interrupt globally

				while(1){					  
						   						  						 
						   					     
							
							if(! (BUTTON_PIN  &  1<<PC2 ) ){ // the SW1 is pressed
								 flag_1 = 1;
								    }//end of if
								
							while(! (BUTTON_PIN  &  1<<PC3 ) ){// the SW1 is pressed more thane 1 second									
									POWER_FLAG ++;
									if(POWER_FLAG > BUTTON_HOLD_MS){// Check if button has been pressed enough
										
											   digit_PORT   |= (1<<PD3) | (1<<PD2) | (1<<PD1) | (1<<PD0);  //Port D[3,2,1,0] set High
											   segment_Code(60,1);
											   _delay_ms(1000);
											   POWER_OFF();
									          }//end of if
							        else{								       
									        // main code here //
											digit_PORT   |= (1<<PD3) | (1<<PD2) | (1<<PD1) | (1<<PD0);  //Port D[3,2,1,0] set High
											segment_Code(20,1);		 	
								                  }//end of else
							       }//end of while	
								   
								   			   	  	 	 		    
				  }//end of while		                     	     						         
    return 0;
}//end of main
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
//Timer/Counter Control
//Register – TCCR0
	
//001         clkI/O/(No prescaling)
//010         clkI/O/8 (From prescaler)
//011         clkI/O/64 (From prescaler)
//100         clkI/O/256 (From prescaler)
//101         clkI/O/1024 (From prescaler)
	
//Timer/Counter Control
//Register – TCCR1B

//001         clkI/O/1 (No prescaling)
//010         clkI/O/8 (From prescaler)
//011         clkI/O/64 (From prescaler)
//100         clkI/O/256 (From prescaler)
//101         clkI/O/1024 (From prescaler)

//Timer/Counter Control
//Register – TCCR2

//001           clkT2S/(No prescaling)
//010           clkT2S/8 (From prescaler)
//011           clkT2S/32 (From prescaler)
//100           clkT2S/64 (From prescaler)
//101           clkT2S/128 (From prescaler)
//110           clkT2S/256 (From prescaler)
//111           clkT2S/1024 (From prescaler)

// 8-bit timer ISR
   * 1/8MHz * (2^8) =    32us
   * 8/8MHz * (2^8) =    256us
   * 64/8MHz * (2^8) =   2.048ms
   * 256/8MHz * (2^8) =  8.192ms
   * 1024/8MHz * (2^8) = 32.768ms
   
// 16-bit timer ISR
	* Enable to increment count on timer overflow @ 2^16 (16-bit timer)
	* Timer Resolution = prescaler/input frequency
	* 1/8MHz * (2^16) =    8.192ms
	* 8/8MHz * (2^16) =    65.536ms
	* 64/8MHz * (2^16) =   524.288ms
	* 256/8MHz * (2^16) =  2.097s
	* 1024/8MHz * (2^16) = 8.389s   
*/
//-------------------------------------------------------------------------------------------------------------
void WDT_off(void){
	
	// reset WDT 
	// Write logical one to WDCE and WDE
	WDTCR |= (1<<WDCE) | (1<<WDE);
	// Turn off WDT
	WDTCR = 0x00;
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
	PORTB = 0X00;
	PORTD = 0X00;
	sei(); // Enable interrupts
	sleep_cpu();
	sleep_disable();	
}
//-------------------------------------------------------------------------------------------------------------
//EXTERNAL INTERRUPT
void external_Interrupt(void){

	  		GICR=(1<<INT1)|(1<<INT0);// Enable INT0 and INT1
	  		MCUCR |= (1 << ISC00);// INT0 is executed on every edge
	  		MCUCR |= (1 << ISC10);// INT1 is executed on every edge
	  		DDRD &= ~(1 << PD2);// Clear the PD2 pin  PD2 (INT0 pin) is now an input
	  		DDRD &= ~(1 << PD3);// Clear the PD3 pin  PD3 (INT1 pin) is now an input	
}
//-------------------------------------------------------------------------------------------------------------
//Interrupt Service Routine (ISR) for INT0
ISR(INT0_vect){
	//Toggle PD5
	RL1_PORT |=  (1<<RL1_BIT);
	RL2_PORT &= ~(1<<RL2_BIT);
}
//-------------------------------------------------------------------------------------------------------------
//Interrupt Service Routine (ISR) for INT1
ISR(INT1_vect){
	//Toggle PD7
	RL1_PORT &= ~(1<<RL1_BIT);
	RL2_PORT |=  (1<<RL2_BIT);	
}
//-------------------------------------------------------------------------------------------------------------
//OVERFLOW INTERRUPT
void overflow_Interrupt(void){
//-------------------------------------------------------------------------------------------------------------	
			TCNT0 =0;
			TCNT1 =0;
			TCNT2 =0;
			
	        TCCR0  |= (1 << CS00);// Start Timer0 with prescalar = 8 (1MHz clock gives overflow interrupts every 2 ms)
			TCCR1B |= (1 << CS10);// Start Timer1 with prescalar = 8 (1MHz clock gives overflow interrupts every 2 ms)
			TCCR2  |= (1 << CS20);// Start Timer2 with prescalar = 8 (1MHz clock gives overflow interrupts every 2 ms)
			
			TIMSK  |= (1<<TOIE0);// Enable Timer 0 overflow interrupt
			TIMSK  |= (1<<TOIE1);// Enable Timer 1 overflow interrupt
			TIMSK  |= (1<<TOIE2);// Enable Timer 2 overflow interrupt		
}
//-------------------------------------------------------------------------------------------------------------
// Interrupt subroutine for Timer0 Overflow interrupt
ISR(TIMER0_OVF_vect){
	
     if(flag_1==1){     // the SW1 is pressed           
		 
		   	PORTD  = 0xff;
			flag_2 = 1;
			_delay_ms(1000);
		}
    			  			 			 
}
//-------------------------------------------------------------------------------------------------------------
// Interrupt subroutine for Timer1 Overflow interrupt
ISR(TIMER1_OVF_vect){
	
    if (flag_2==1){
				    
			PORTB = 0xff;
			PORTD = 0x00;
			_delay_ms(1000);
			flag_1 = 0;
			flag_3 = 1;

                 }
	
}
//-------------------------------------------------------------------------------------------------------------
// Interrupt subroutine for Timer2 Overflow interrupt
ISR(TIMER2_OVF_vect){
	
	if(flag_3 == 1){
		 PORTB  = 0x00;
		 flag_2 = 0;
		 flag_3 = 0;
	            }	
}
//-------------------------------------------------------------------------------------------------------------
//COMPARE MATCH INTERRUPT
void Timer1_compare_Match_interrupt(void){
	
		//set timer1 interrupt at 1Hz
		TCCR1A = 0;// set entire TCCR1A register to 0
		TCCR1B = 0;// same for TCCR1B
		TCNT1  = 0;//initialize counter1 value to 0	
		TCCR1A |= (1 << WGM12);// CTC mode
		//TCCR1A |= (1 << COM1A0);// CTC mode, toggle OC1A on compare match
		//TCCR1B |= (1 << COM1B0);// Toggles OC1B pin each cycle through
        TCCR1B |= (1 << CS11)|(1 << CS10);// set up timer with prescaler = clock/64 // Set CS12 bits for 256 prescaler=clock/256		
		TIMSK |= (1 << OCIE1A);// Output Compare A Match Interrupt Enable
		TIMSK |= (1 << OCIE1B);// Output Compare B Match Interrupt Enable
		// set compare match register for 1hz increments. = (16*10^6) / (1*1024) - 1 (must be <65536)
		OCR1A = 15625;// initialize compare value.around 1000ms delay
		OCR1B = 7812;// initialize compare value.around 500ms delay
		//OCR1B = 125;//initialize compare value.around 1ms delay	
		//OCR1A = 3906; 
		// F_Timer = F_clk/Prescaler
		// = 8'000'000 / 1024 = 7812.5
		// F_PDx = F_Timer/ 2 / OCR1A
		// = 7812.5 / 2 / 3906 = 1.00006Hz
}
//-------------------------------------------------------------------------------------------------------------
// Interrupt subroutine for Timer1 Compare Match A Interrupt
ISR(TIMER1_COMPA_vect) {	
	LED_PORT ^= (1<< LED_BIT);		
}
//-------------------------------------------------------------------------------------------------------------
// Interrupt subroutine for Timer1 Compare Match B Interrupt
ISR(TIMER1_COMPB_vect) {
	BUZ_PORT ^= (1<< BUZ_BIT) ;
}
//-------------------------------------------------------------------------------------------------------------
void Timer2_compare_Match_interrupt(void){
	
			//set timer2 interrupt at 1Hz
			TCCR2 = 0;// set entire TCCR2 register to 0
			TCNT2 = 0;//initialize counter2 value to 0
			TCCR2 |= (1 << WGM21);// CTC mode
			TCCR2 |= (1 << COM20);// CTC mode, toggle OCn on compare match . page 116 of ATmega8 data sheet.
			TCCR2 |= (1 << CS21);// set up timer with prescaler. page 117 of ATmega8 data sheet. F_Timer = F_clock/Prescaler.
			TIMSK |= (1 << OCIE2);// Output Compare  Match Interrupt Enable.page 120 of ATmega8 data sheet.

			// set compare match register for delay.  most be < (F_clock/Prescaler - 1)
			OCR2 = 7812;//  initialize compare value. delay = F_Timer /2 /OCR2 
			//OCR2 = 125;// initialize compare value. delay = F_Timer /2 /OCR2 						
			//if OCR2 = 3906 , clock = 8MHz , prescaler = 1024 ===>>
			// = 8'000'000 /1024 = 7812.5
			// = 7812.5 /2 /3906 = 1.00006Hz
				
}
//-------------------------------------------------------------------------------------------------------------
ISR(TIMER2_COMP_vect){
				
}
*/
//-------------------------------------------------------------------------------------------------------------
//Question: When OC1A toggle?
//Answer: when TCNT1 == OCR1A
//Question: when OC1B toggle?
//Answer: when TCNT1 == OCR1B
//-------------------------------------------------------------------------------------------------------------

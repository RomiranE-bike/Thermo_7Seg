/*
 * adc+7seg+interrupt.c
 * ver.01
 * Created: 12/24/2023 15:57:40
 * edited : me
 */ 
//-----------------------------------------------------------------------------------------------
// File Name    : 'main.c'
// Title        : ADC single conversion within interrupt after conversion
//-----------------------------------------------------------------------------------------------
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
//-----------------------------------------------------------------------------------------------
uint8_t i = 0;                // counter variable
uint8_t previousReading  = 1; // holds last 8 pin reads from pushbutton
uint8_t buttonWasPressed = 1; // keeps track of debounce d button state, 0 false, 1 true
//-----------------------------------------------------------------------------------------------
#define BUTTON_PORT PORTC           // PORTx - register for button output 
#define BUTTON_PIN  PINC            // PINx - register for button input 
#define BUTTON_BIT  PC2             // bit for button input 
//-----------------------------------------------------------------------------------------------
#define LED_DDR  DDRD               // LED data direction register 
#define LED_PORT PORTD              // PORTx - register for LED output 
#define LED_BIT  PD4                // bit for LED output 
#define RL1_BIT  PD6                // bit for LED output 
//-----------------------------------------------------------------------------------------------
void key_function(void);
void timer1_init(void);
//----------------------------------------------------------------------------------------------- 
int main(void){
	
		LED_DDR  = 0xFF; // set LED pin as digital output
		LED_PORT |= (1 << LED_BIT);// led is OFF initially

		BUTTON_PORT |= (1 << BUTTON_BIT);// turn on internal pull-up resistor for the switch	
		
		timer1_init();
		//initTimer();	 
						while(1){	
							
                               //key_function();

		                     	}				
    return 0;
}
//-------------------------------------------------------------------------------------------------------------
void key_function(void){						 						 														
						 									
					if(! (BUTTON_PIN  &  1<<BUTTON_BIT ) ){     // the button is pressed 
						LED_PORT ^= (1<<LED_BIT) ;              // toggle LED
						_delay_ms(1000);
					}	
}

//-------------------------------------------------------------------------------------------------------------
/*
ISR(TIMER1_COMPA_vect)
{
	if (!(PINC & (1 << PC2)) == previousReading)   // if current button pin reading doesn't equal previousReading 200us ago,
	{
		if(!buttonWasPressed)                     // and button wasn't pressed last time
		{
			if(i < 15)                            // increment i from 0 to 15
			i++;
			else
			i = 0;
			PORTC = i | (1 << PC2);               // set PORTB to i, along with PB4 bit to keep pull up set
			buttonWasPressed = 1;                 // set debounce d button press state to 1
			                
			PORTD ^= (1 << PD4);
			_delay_ms(1000);				
		}
		else
		buttonWasPressed = 0;  	                  // button has been let go, reset buttonWasPressed
	}
	previousReading = ~(PINC &  1<< PC2);        // set previous reading to current reading for next time
}
*/
//-------------------------------------------------------------------------------------------------------------
ISR(TIMER1_COMPA_vect) {
	LED_PORT ^= (1<< LED_BIT) ;
}
//-------------------------------------------------------------------------------------------------------------
ISR(TIMER1_COMPB_vect) {
	LED_PORT ^= (1<< RL1_BIT) ;
}
//-------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
//Question: When OC1A toggle?
//Answer: when TCNT1 == OCR1A
//Question: when OC1B toggle?
//Answer: when TCNT1 == OCR1B
//-------------------------------------------------------------------------------------------------------------
void timer1_init(void){
	//set timer1 interrupt at 1Hz
	TCCR1A = 0;// set entire TCCR1A register to 0
	TCCR1B = 0;// same for TCCR1B
	TCNT1  = 0;//initialize counter value to 0
	
	//TCCR1A |= (1 << COM1B0);            // Toggles OC1B pin each cycle through	
	TCCR1B |= (1 << WGM12);             // CTC mode
	TCCR1B |= (1 << CS11)|(1 << CS10);  // set up timer with prescaler = clock/64 // Set CS12 bits for 256 prescaler=clock/256
	
	TIMSK |= (1 << OCIE1A);             // Output Compare A Match Interrupt Enable
	TIMSK |= (1 << OCIE1B);		        // Output Compare B Match Interrupt Enable

// set compare match register for 1hz increments // = (16*10^6) / (1*1024) - 1 (must be <65536)	
	OCR1A = 15625;                      // initialize compare value // around 1000ms delay
	OCR1B = 7812;                       // initialize compare value // around 500ms delay
	sei();
}
//-------------------------------------------------------------------------------------------------------------

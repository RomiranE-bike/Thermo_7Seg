/*
 * relay.h
 *
 * Created: 1/20/2024 16:01:54
 *  Author: User
 */ 
#ifndef RELAY_H_
#define RELAY_H_
//----------------------------------------------------------------------------
#include <avr/io.h>
#include <util/delay.h>
//----------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------
int        f_beep=250;     //beep frequency :250 cycle  if  clock set to 8MHZ/ 500 cycle if clock set to 1MHZ
//const int  d_beep=250;      //beep delay :250us if clock set to 4MHZ /50us if clock set to 1MHZ
//int        d_blink=100;     //beep delay :100 ms if clock set to 4MHZ /100 ms if clock set to 1MHZ
//----------------------------------------------------------------------------
void set_Relay(int mili_volt_0,int T1);
void set_Relay(int mili_volt_1,int T2);
void beep();
void blink();
//----------------------------------------------------------------------------
// Port and pin definitions for sell
#define Relay_DDR		    DDRD
#define Relay_PORT	        PORTD
//----------------------------------------------------------------------------
#define Relay_1         (1 << PD4)
#define Relay_2         (1 << PD5)
//---------------------------------------------------------------------------------------------------
#define LED             (1 << PD6)
#define Buzzer          (1 << PD7)
//----------------------------------------------------------------------------
void set_Relay1(int mili_volt_0,int T1){
	
			Relay_DDR  |= (Relay_1);                   //  select output pins (Relay_1|Relay_2)				
										   
				 if (mili_volt_0 > T1 ){            
						Relay_PORT &=~( Relay_1);              //relay1 off
						 }
				 else if(mili_volt_0 < T1 ) { 
							Relay_PORT |= ( Relay_1);          //relay1 ON
							 }
}
//---------------------------------------------------------------------------------------------------
void set_Relay2(int mili_volt_1,int T2){
	
			Relay_DDR  |= (Relay_2);                   //  select output pins (Relay_1|Relay_2)
	
				if(mili_volt_1 > T2 ) {
					Relay_PORT &= ~(Relay_2);          //relay2 off
					}
				else if(mili_volt_1 < T2 ) {
					Relay_PORT |= ( Relay_2);          //relay2 ON
				   }
	
}
//---------------------------------------------------------------------------------------------------
void blink(){                      // blinking routine
	
	Relay_DDR  |= (LED);           //  select output pins (LED)
	
	Relay_PORT |= (LED);           // Toggle LED
	_delay_ms(100);                // Delay
	Relay_PORT &=~(LED);           // Toggle LED
	_delay_ms(100);                // Delay
}
//---------------------------------------------------------------------------------------------------
void beep(){                         // Beeping routine

	int i;
	Relay_DDR  |= (Buzzer);           //  select output pins (Buzzer)
	
	for(i=0;i<f_beep;i++){            // Loop
		Relay_PORT |= (Buzzer);       // Toggle BUZZER
		_delay_us(250);               // Delay
		Relay_PORT &=~(Buzzer);       // Toggle BUZZER
		_delay_us(250);               // Delay
	};
}
//---------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------
#endif /* RELAY_H_ */
//----------------------------------------------------------------------------
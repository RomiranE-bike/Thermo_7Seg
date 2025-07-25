/*
 * beep.h
 *
 * Created: 1/31/2024 14:57:54
 *  Author: me
 */ 
//---------------------------------------------------------------------------------------------------
#ifndef BEEP_H_
#define BEEP_H_
//---------------------------------------------------------------------------------------------------
#include <avr/io.h>
#include <util/delay.h>
//---------------------------------------------------------------------------------------------------
#define action_DDR     DDRD
#define action_PORT    PORTD

#define LED_BIT  PD6// bit for LED output
#define BUZ_BIT  PD7// bit for LED output

//---------------------------------------------------------------------------------------------------
#define   f_beep   500    //beep frequency :500 cycle  if  clock set to 4MHZ. 250 cycle if clock set to 1MHZ
#define   d_beep   500    //beep delay :500us if clock set to 4MHZ. 250us if clock set to 1MHZ
#define   d_blink  300    //beep delay :300 ms if clock set to 4MHZ. 100 ms if clock set to 1MHZ
//---------------------------------------------------------------------------------------------------
void beep();
void blink();
void alarm(void);
//---------------------------------------------------------------------------------------------------
// blinking routine//
void blink(){                   
 
    action_PORT  |= 1<<LED_BIT;// Toggle LED          
	 _delay_ms(d_blink);// Delay
	 action_PORT  &=~ (1<<LED_BIT);// Toggle LED
	 _delay_ms(d_blink);// Delay
}
//---------------------------------------------------------------------------------------------------
// Beeping routine//
void beep(){
                        
    for(int i=0;i<f_beep;i++){// Loop
        action_PORT ^= 1<<BUZ_BIT;// Toggle BUZZER
         _delay_us(d_beep);// Delay
        }
}
//---------------------------------------------------------------------------------------------------
void alarm(void){
	for(int i=0;i<50;i++){		
		beep();
		blink();
		_delay_ms(10);
	}
}
//-------------------------------------------------------------------------------------------------------------
#endif /* BEEP_H_ */
//---------------------------------------------------------------------------------------------------
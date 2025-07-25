/*
 * timer_interrupt.h
 *
 * Created: 2/1/2024 16:56:42
 *  Author: me
 */ 
//-------------------------------------------------------------------------------------------------------------
#ifndef TIMER_INTERRUPT_H_
#define TIMER_INTERRUPT_H_
//-------------------------------------------------------------------------------------------------------------
void POWER_OFF(void);
void disable_interrupt(void);
void external_Interrupt(void);// INT0 and INT1
void timer0_overflow_Interrupt(void);// timer 0
void timer1_overflow_Interrupt(void);// timer 1
void timer2_overflow_Interrupt(void);// timer 2
void timer1_compare_Match_interrupt(void);// compare 1A and 1B
void timer2_compare_Match_interrupt(void);// compare 2
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
	cli(); //irq's off
	wdt_enable(WDTO_15MS); //wd on,15ms
	while(1); //loop
	
	// reset WDT 
	// Write logical one to WDCE and WDE
	WDTCR |= (1<<WDCE) | (1<<WDE);
	// Turn off WDT
	WDTCR = 0x00;
}
*/
//-------------------------------------------------------------------------------------------------------------
// interrupt function recall//
/*
//timer0_overflow_Interrupt();//refresh seven segment display
//timer1_overflow_Interrupt();//sensing sensor and Ladder resistor
//timer2_overflow_Interrupt();
//timer1_compare_Match_interrupt();
//timer2_compare_Match_interrupt();
//external_Interrupt();
//sei();//   enable interrupt globally
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
//ALL interrupt disable//
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
}
//-------------------------------------------------------------------------------------------------------------
//Interrupt Service Routine (ISR) for INT1
ISR(INT1_vect){	
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
			
}
//-------------------------------------------------------------------------------------------------------------
// Timer1 Compare Match B vector //
ISR(TIMER1_COMPB_vect) {
	
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
				
}
//-------------------------------------------------------------------------------------------------------------
//Question: When OC1A toggle?
//Answer: when TCNT1 == OCR1A
//Question: when OC1B toggle?
//Answer: when TCNT1 == OCR1B
//------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------
#endif /* TIMER_INTERRUPT_H_ */
//-------------------------------------------------------------------------------------------------------------
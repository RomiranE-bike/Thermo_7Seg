#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>


void eeprom_update(int eep_value_1,int eep_value_2){
	 
	display_char(null,E,E,P);
	_delay_ms(100);
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
	_delay_ms(100);
	display_num(read_value_1); //PASS THE VALUE OF 'eeprom_value' FOR DIGIT BREAKUP AND DISPLAY
	_delay_ms(100);
	display_char(null,E,E,2);
	_delay_ms(100);
	display_num(read_value_2); //PASS THE VALUE OF 'eeprom_value' FOR DIGIT BREAKUP AND DISPLAY
	_delay_ms(100);
}

//-------------------------------------------------------------------------------------------------------------

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
	
//-------------------------------------------------------------------------------------------------------------


EEMEM unsigned char colors[2][3]={{1, 2, 3},{4, 5, 6}};

eeprom_write_byte(&colors[0][0], 1); 
eeprom_write_byte(&colors[0][1], 2);
eeprom_write_byte(&colors[0][2], 3);
eeprom_write_byte(&colors[1][0], 4);
eeprom_write_byte(&colors[1][1], 5);
eeprom_write_byte(&colors[1][2], 6);

unsigned char temp;
temp = eeprom_read_byte(&colors[1][0]); 

//-------------------------------------------------------------------------------------------------------------

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

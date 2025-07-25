/*
 * note_formula.c
 *
 * Created: 1/15/2024 21:53:39
 *  Author: User
 */ 

//----------------------------------------------------------------------------------------//
	//NOTE:
	
	//----------------------------------------------------------------------------------------//
	/*
		Digital output = 2n * (Analog Input Voltage)/(Analog Reference Voltage)
		(Digital output * Analog Reference Voltage) / 2n = Analog Input Voltage
		value to degree celcius: Voltage = (2.56/1024) * adc_val;
		Celcius = Voltage/10mV = (2.56/0.01/1024)* adc_val =(256/1024)* adc_val => adc_val/4 (resolution = 1LSB = 0.25 Celcius)
		float Temperature = ((adc_val * 4.88)-0.0027)/10.0;
		degree =  (((( mili_volt * 48)/10) - 2280) / 10);
		mili_volt_0  =  (data_channel[0])/216.5*1000;
		degree =  ((mili_volt * 4.88)-0.0027)/10.0;
		Voltage = (v_refrence * adc_val)/1024 ;
		//-----------------------------------------------------------------------------------------//
		float f;
		unsigned int a;
		a = (data_channel[0]);
		f = a;
		f = f * 0.0048828125;
		//-----------------------------------------------------------------------------------------//
		mili_volt = (adc_code * 5000) >> 16;
		//-----------------------------------------------------------------------------------------//
		float voltage = 0;
		voltage = ADC/204.8*18;//ADC/18.618;
		//-----------------------------------------------------------------------------------------//
		unsigned int number;
		long tlong;
		unsigned int voltage;
		adc_value = ADC_Read(0);  // read data from channel 0
		tlong = (float)adc_value * 0.488768555;
		voltage = tlong;
		VOUT = (R1/(R1+R2) * VIN);
	//-----------------------------------------------------------------------------------------//
	
		Note: This voltmeter is only as accurate as your reference voltage.
		If you want four digits of accuracy, need to measure your AVCC well.
		Measure either AVCC of the voltage on AREF and enter it here.
		
	//-----------------------------------------------------------------------------------------//
	#define REF_VCC 5.053
									// measured division by voltage divider 
	#define VOLTAGE_DIV_FACTOR  3.114
	float voltage;
	voltage = oversample16x() * VOLTAGE_DIV_FACTOR * REF_VCC / 4096;
	//-----------------------------------------------------------------------------------------//
	float voltage;
	voltage = ((float)read_adc(channel) * 4.93) / 1023;
	voltage = (voltage - 2.428) * 10.43478;
	//-----------------------------------------------------------------------------------------//
	//---------------------------------------------------------------------------
	float volt;
	volt = 5.0 * (float)ADC;
	volt = volt / 1023.0;
	volt = volt * 10.0;	// scale by 10 for display 0.x - 5.x
	return ((unsigned int)(volt));
		
	double Amper;
	Amper = 0.074 * (float)volt;
	Amper = fabs(Amper-37.888);
	Amper = Amper * 10.0;
		
	// before return we must scale value by 10 for display
	return (unsigned int)Amper;
	//return ((unsigned int)(fabs(((0.074 * (double)ADC_Count) - 37.888))*10.0));
*/
	//----------------------------------------------------------------------------------------------							
	//-----------------------------------------------------------------------------------------//
	
	//-----------------------------------------------------------------------------------------//
	/*
	#include <LiquidCrystal_I2C.h>
	LiquidCrystal_I2C lcd(0x26,16,2);//address of lcd, column no, row no.
	
	#define CS 5
	#define SO 7
	#define SCK 6
	
	void setup()
	{
		lcd.init(); // initialize the lcd
		lcd.backlight(); //Backlight ON
	}
	
	void loop() {
		float temp_C = Thermocouple_read();
		if (isnan(temp_C))
		{
			lcd.setCursor(0,0);
			lcd.print("Connect");
			lcd.setCursor(0,1);
			lcd.print("Thermocouple");
			delay(1000);
			loop();
		}
		float Temp_fn = (temp_C*1.8)+32;
		lcd.setCursor(0,0);
		lcd.print("Termocouple Temp");
		lcd.setCursor(0,1);
		lcd.print(temp_C,2);
		lcd.print((char)223);
		lcd.print("C");
		lcd.setCursor(8,1);
		lcd.print(Temp_fn,2);
		lcd.print((char)223);
		lcd.print("F");
		delay(1000);
	}
	
	double Thermocouple_read() {
		lcd.clear();
		uint16_t v_out;
		pinMode(CS, OUTPUT);
		pinMode(SO, INPUT);
		pinMode(SCK, OUTPUT);
		
		digitalWrite(CS, LOW);
		delay(1);
		
		v_out = shiftIn(SO, SCK, MSBFIRST);
		v_out <<= 8;
		v_out |= shiftIn(SO, SCK, MSBFIRST);
		
		digitalWrite(CS, HIGH);
		if (v_out & 0x4)
		{
			//Thermocouple is disconnected
			return NAN;
		}
		
		// The lower three bits (0,1,2) are discarded status bits
		v_out >>= 3;
		
		// The remaining bits are the number of 0.25 degree (C) counts
		return v_out*0.25;
	}
*/
/*
const int thermocouple_input = A1;	// AD595 O/P pin 

void setup() {
	Serial.begin(9600);	// Define baud rate for serial communication 
}

void loop() {
	int adc_val;
	float temperature;
	adc_val = analogRead(thermocouple_input);
	temperature = ( ((adc_val * 4.88) - 0.0027 ) / 10.0 );
	Serial.print("Temperature = ");
	Serial.print(temperature);
	Serial.print("\n\n");
	delay(1000);
}
*/
/*
 * FinalProject.cpp
 *
 * Created: 4/25/2025 1:32:14 PM
 * Author : logan
 */ 

#define F_CPU 16000000 // 16 MHz
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
//TWI constants
#define TWI_START 0xA4 //(enable=1, start=1, interrupt flag=1)
#define TWI_STOP 0x94 //(enable=1, stop=1, interrupt flag=1)
#define TWI_SEND 0x84 //(enable=1, start=0, interrupt flag=1)
//LCD Signals
//D7 D6 D5 D4 BL E RW RS
#define LCD_ADDRESS 0x4E //device address (0x27) in 7 MSB + W (0) in LSB
#define LCD_DISABLE 0x04
#define LCD_ENABLE 0x00
#define LCD_WRITE 0x00
#define LCD_READ 0x01
#define LCD_RS 0x01
#define LCD_BL 0x08
//LCD Commands (D0-D7 in Datasheet)
#define CLEAR_DISPLAY 0x01 //need to wait 2ms after this call (perdatasheet)
#define CURSOR_HOME 0x02 //need to wait 2ms after this call (perdatasheet)
#define SET_ADDRESS 0x80
volatile uint8_t rising_edges = 0x00;
volatile uint8_t falling_edges =0x00;
volatile uint8_t switch_states =0x00;
volatile uint8_t buttons = 0x00;
volatile uint8_t buttonIndex = 0;
volatile uint8_t lockstate = 0;
uint16_t countDownNumber = 0;
uint8_t countDownEnable = 0;
//declare buffer array
const uint8_t kBufferLength = 8;
volatile uint8_t switch_array[kBufferLength]={0};
//stepper state machine variable
int8_t stepperState=0;
int8_t stepperDirection=0;

//alarm code
int8_t alarmcode[] = {2, 2, 1};
int8_t currentCode[] = {0, 0, 0};
int8_t armedCode[] = {2, 3, 3};
int8_t alarmIndex = 0;
int8_t currentButton = 0;
int8_t enableCountDown = 0;
//10 second countdown
int8_t countDownIndex = 10;
int16_t countDownSecond = 0;

int8_t armed = 0;
int8_t alarmEnable = 0;
//Sample switches at particular interval
void softwareDebounce(uint8_t sampleData){
	static uint8_t sample_index= 0; //Index used to store switchsamples in array
	uint8_t stable_high = 0xFF; //Initialize temporarystable_high all high
	uint8_t stable_low = 0; //Initialize temporarystable_low all low
	switch_array[sample_index] = sampleData; //Store switch sample from SPI new_data in array
	//Loop through all historical switch samples to check for stable highs and lows
	for (uint8_t i=0;i<kBufferLength;i++){
		//"And" for stable high (all 1's will produce "1" for stable high)
		stable_high &= switch_array[i];
		//"Or" for stable low (all 0's will produce "0" for stable low)
		stable_low |= switch_array[i];
	}
	rising_edges = (~switch_states)&stable_high;
	//Detect Rising Edges
	falling_edges = switch_states&(~stable_low);
	//Detect Falling Edges
	switch_states = stable_high|(switch_states&stable_low);
	//Update switch states
	//Update sample index and wrap if necessary
	if(++sample_index>=kBufferLength)
	sample_index = 0;//wrap
}
void TWI(unsigned char address, unsigned char data)
{
	TWCR = TWI_START; //send start condition
	while(!(TWCR & (1<<TWINT))){} //wait for start condition to transmit
	TWDR = address; //send address to get device attention
	TWCR = TWI_SEND; //Set TWINT to send address
	while(!(TWCR & (1<<TWINT))){} //wait for address to go out
	TWDR = data; //send data to address
	TWCR = TWI_SEND; //Set TWINT to send address
	while(!(TWCR & (1<<TWINT))){} //wait for data byte to transmit
	TWCR = TWI_STOP; //finish transaction
}
void LCD_Display(unsigned char data)
{
	// Put in character data upper bits while keeping enable bit high
	TWI(LCD_ADDRESS,(data & 0xF0)|LCD_DISABLE|LCD_WRITE|LCD_BL|LCD_RS);
	// Pull enable bit low to make LCD display the data
	TWI(LCD_ADDRESS,(data & 0xF0)|LCD_ENABLE|LCD_WRITE|LCD_BL|LCD_RS);
	// Put in character data lower bits while keeping enable bit high
	TWI(LCD_ADDRESS,((data<<4) & 0xF0)|LCD_DISABLE|LCD_WRITE|LCD_BL|LCD_RS);
	// Pull enable bit low to make LCD display the data
	TWI(LCD_ADDRESS,((data<<4) & 0xF0)|LCD_ENABLE|LCD_WRITE|LCD_BL|LCD_RS);
}
void LCD_Command(unsigned char command)
{
	// Put in command data upper bits while keeping enable bit high
	TWI(LCD_ADDRESS,(command &0xF0)|LCD_DISABLE|LCD_WRITE|LCD_BL);
	// Pull enable bit low to make LCD process the command
	TWI(LCD_ADDRESS,(command &0xF0)|LCD_ENABLE|LCD_WRITE|LCD_BL);
	// Put in command data lower bits while keeping enable bit high
	TWI(LCD_ADDRESS,((command<<4) & 0xF0)|LCD_DISABLE|LCD_WRITE|LCD_BL);
	// Pull enable bit low to make LCD process the command
	TWI(LCD_ADDRESS,((command<<4) & 0xF0)|LCD_ENABLE|LCD_WRITE|LCD_BL);
}

void clearNumbers() {
	LCD_Command(SET_ADDRESS|0x45);
	LCD_Display(' ');
	LCD_Display(' ');
	LCD_Display(' ');
	LCD_Command(SET_ADDRESS|0x45);
	alarmIndex = 0;
	currentButton = 0;
	currentCode[0] = 0;
	currentCode[1] = 0;
	currentCode[2] = 0;
}


void lockStateMachine(){
	if(!(PIND&(1<<PIND5))) {
		currentButton = 1;
	} else if(!(PIND&(1<<PIND6))) {
		currentButton = 2;
	} else if(!(PIND&(1<<PIND7))) {
		currentButton = 3;
	}
	if(alarmcode[alarmIndex] == currentButton && armed == 0) {
		currentCode[alarmIndex] = currentButton;
		alarmIndex++;
		LCD_Display('0' + currentButton);
		if(alarmIndex == 3) {
			LCD_Command(SET_ADDRESS|0x4E);
			LCD_Display('1'); 
			LCD_Display('0');
			enableCountDown = 1;
		}
		
	} else if(armedCode[alarmIndex] == currentButton && armed == 1) {
		currentCode[alarmIndex] = currentButton;
		alarmIndex++;
		LCD_Display('0' + currentButton);
		if(alarmIndex == 3) {
			LCD_Command(SET_ADDRESS|0x4E);
			LCD_Display('1');
			LCD_Display('0');
			enableCountDown = 1;
		} } else {
			if(armed == 1) {
				alarmEnable = 1;
			}
		clearNumbers();
	}
	
	
}
	


void stepperStateMachine() {
	//STATE MACHINE FOR UNL2003 FULL STEP MODE
	switch(stepperState){
		//STEP AB
		case 0:
		PORTC=0x03;
		stepperState=stepperState+stepperDirection;
		break;
		//STEP BC
		case 1:
		PORTC=0x06;
		stepperState=stepperState+stepperDirection;
		break;
		//STEP CD
		case 2:
		PORTC=0x0C;
		stepperState=stepperState+stepperDirection;
		break;
		//STEP DA
		case 3:
		PORTC=0x09;
		stepperState=stepperState+stepperDirection;
		break;
	}
	//CHECK STEPPER STATE OVERFLOW
	if (stepperState>3){
		stepperState=0;
	}
	else if (stepperState<0){
		stepperState=3;
	}
}

void rotateMotor() {
	if(armed == 0) {
		stepperDirection = 1;
		} else {
		stepperDirection = -1;
	}
	for(uint64_t i = 0; i < 512; i++)	{
		stepperStateMachine();
		_delay_ms(10);
	}
	stepperDirection = 0;
	
}
void displayLockState() {
	LCD_Command(SET_ADDRESS|0x00);
	if(armed) {
		LCD_Display('A');
		LCD_Display('R');
		LCD_Display('M');
		LCD_Display('E');
		LCD_Display('D');
		LCD_Display(' ');
		LCD_Display(' ');
		LCD_Display(' ');
	} else {
		LCD_Display('D');
		LCD_Display('I');
		LCD_Display('S');
		LCD_Display('A');
		LCD_Display('R');
		LCD_Display('M');
		LCD_Display('E');
		LCD_Display('D');
	}
}
	
ISR(TIMER1_COMPA_vect){
	buttons = PIND&0b11100000;
	softwareDebounce(buttons);
	
	if((falling_edges&0b11100000)>>5){
		lockStateMachine();
	}
	
	if(enableCountDown == 1) {
		countDownSecond++;
		if(countDownSecond > 1000) {
			countDownSecond = 0;
			countDownIndex--;
			LCD_Command(SET_ADDRESS|0x4E);
			LCD_Display(' ');
			LCD_Display('0' + countDownIndex);
			if(countDownIndex == 0) {
				enableCountDown = 0;
				countDownIndex = 10;
				rotateMotor();
				PORTB ^= (1<<PORTB1);
				PORTB ^= (1<<PORTB2);
				armed ^= 1;
				displayLockState();
				clearNumbers();
			}
		}
	}
	if(alarmEnable) {
		PORTB |= (1<<PORTB0);
		countDownSecond++; 
		if(countDownSecond > 255) {
			PORTB ^= (1<<PORTB1);
		}
	}
}


int main(void)
{
    //CONFIGURE IO
    DDRD = (0<<PORTD5)|(0<<PORTD6)|(0<<PORTD7);
    //Configure Bit Rate (TWBR and TWSR)
    TWBR = 18; //TWBR=18
    TWSR = (0<<TWPS1)|(1<<TWPS0); //PRESCALER = 1
    //Configure TWI Interface (TWCR)
    TWCR = (1<<TWEN);
    //INITIALIZE LCD
    TWI(LCD_ADDRESS,0x30|LCD_DISABLE|LCD_WRITE|LCD_BL); // (data length of 8, number of lines=2, 5x8 digit space, load data)
    TWI(LCD_ADDRESS,0x30|LCD_ENABLE|LCD_WRITE|LCD_BL); // (clock in data)
    _delay_ms(15);
    TWI(LCD_ADDRESS,0x30|LCD_DISABLE|LCD_WRITE|LCD_BL); // (data length of 8, number of lines=2, 5x8 digit space, load data)
    TWI(LCD_ADDRESS,0x30|LCD_ENABLE|LCD_WRITE|LCD_BL); // (clock in data)
    _delay_ms(4.1);
    TWI(LCD_ADDRESS,0x30|LCD_DISABLE|LCD_WRITE|LCD_BL); // (data length of 8, number of lines=2, 5x8 digit space, load data)
    TWI(LCD_ADDRESS,0x30|LCD_ENABLE|LCD_WRITE|LCD_BL); // (clock in data)
    _delay_ms(4.1);
    TWI(LCD_ADDRESS,0x20|LCD_DISABLE|LCD_WRITE|LCD_BL); // (data length of 4, number of lines=2, 5x8 digit space, load data)
    TWI(LCD_ADDRESS,0x20|LCD_ENABLE|LCD_WRITE|LCD_BL); // (clock in data)
    TWI(LCD_ADDRESS,0x20|LCD_DISABLE|LCD_WRITE|LCD_BL); // (load data)
    TWI(LCD_ADDRESS,0x20|LCD_ENABLE|LCD_WRITE|LCD_BL); // (clock in data)
    //HOME CURSOR
    TWI(LCD_ADDRESS,0x80|LCD_DISABLE|LCD_WRITE|LCD_BL); // (load data)
    TWI(LCD_ADDRESS,0x80|LCD_ENABLE|LCD_WRITE|LCD_BL); // (clock in data)
    //CLEAR DISPLAY AND WAIT TO FINSIH
    LCD_Command(CLEAR_DISPLAY);
    _delay_ms(2);
    LCD_Command(0x0C); //Display no cursor
    LCD_Command(0x06); //Automatic Increment
    LCD_Command(SET_ADDRESS|0x00);
    LCD_Display('D');
    LCD_Display('I');
    LCD_Display('S');
    LCD_Display('A');
    LCD_Display('R');
    LCD_Display('M');
	LCD_Display('E');
	LCD_Display('D');
    LCD_Command(SET_ADDRESS|0x40);
    LCD_Display('C');
    LCD_Display('O');
    LCD_Display('D');
    LCD_Display('E');
    LCD_Display(':');
	
	PORTB |= (1<<PORTB2);
    //Configure Timer 1
    TCCR1A=(0<<WGM11)|(0<<WGM10);
    TCCR1B=(0<<CS12)|(0<<CS11)|(1<<CS10)|(0<<WGM13)|(1<<WGM12);
    TIMSK1=(1<<OCIE1A);
    OCR1A=15999;
    sei();
	//timer 1 = 1 ms
    /* Replace with your application code */
    while (1)
    {
    }
    }
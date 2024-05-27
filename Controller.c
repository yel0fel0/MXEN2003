//include this .c file's header file
#include "Controller.h"
#include <avr/interrupt.h>
#define DEBOUNCE_PERIOD 800


//File scope variables
static char serial_string[200] = {0};
volatile uint8_t dataByte1=0, dataByte2=0, dataByte3=0, dataByte4=0;		// data bytes received
volatile bool new_message_received_flag=false;
volatile bool button_state = false;

///Sensor Values into Distance
static uint8_t R_dist = 0;
static uint8_t F_dist = 0;
static uint8_t L_dist = 0;

int main(void)
{
	// Initialisation for serial communication
	serial0_init(); 	// terminal communication with PC
	serial2_init();		// microcontroller communication to/from another Arduino
	// or loopback communication to same Arduino

	adc_init(); //initialise Analog to Digital
	_delay_ms(20); //it's a good idea to wait a bit after your init section
	lcd_init(); //initialise ldc and read the hd44780 settings files and update all the reg so it knows what to use.
	_delay_ms(20);
	milliseconds_init();	//initialise miliseconds timer

	// Var declare
	uint8_t sendDataByte1=0, sendDataByte2=0, sendDataByte3=0, sendDataByte4=0;		// data bytes sent
	uint32_t current_ms=0, last_send_ms=0;			// used for timing the serial send
	
	// USART communication interrupt
	UCSR2B |= (1 << RXCIE2); // Enable the USART Receive Complete interrupt (USART_RXC)

	// Enable Interrupt for Software Debounce for Autonomous mode
	DDRD &= ~(1<<PD2);   	// Clear bit ...Set port D = 1.0 => PD0 or INT0 as input mode
	PORTD |= (1<<PD2);    	// Enable pull-up resistor with debounce RC debouce Hardware Interrupt
	EICRA |= (1<<ISC21);    // Set bit 	- Falling edge trigger when ISCn1 / ISCn0 = 1 / 0
	EICRA &= ~(1<<ISC20);   // Clear bit
	EIMSK |= (1<<INT2);   	// Interrupt INT0 enable in mask
	sei();    				//Set enable interrupt.


	while(1)
	{
		current_ms = milliseconds_now();		//current time in ms.
		char lcd_string[32] = {0};
		uint8_t RJoyXC = (adc_read(1)>>2);		//Read in analog voltage from joystick
		uint8_t RJoyYC = (adc_read(0)>>2);		//Read in analog voltage from joystick
		uint8_t LJoyXC = (adc_read(14)>>2);		//Read in analog voltage from joystick
		uint8_t LJoyYC = (adc_read(15)>>2);		//Read in analog voltage from joystick

		if (button_state)
		{
			serial0_print_string("toggled\n");
		}
		// if (LJoyYC > 140)
		// {
		// 	RJoyXC = RJoyXC / 2;
		// 	RJoyYC = RJoyYC / 2;
		// }
		// Joystick Deadzone
		if (abs(RJoyXC - 126) > 3)
		RJoyXC = RJoyXC;
		else
		RJoyXC = 126;

		if (abs(RJoyYC - 126) > 3)
		RJoyYC = RJoyYC;
		else
		RJoyYC = 126;

		if (abs(LJoyXC - 126) > 3)
		LJoyXC = LJoyXC;
		else
		LJoyXC = 126;
		
		if (RJoyXC > 252)
		RJoyXC = 252;
		if (RJoyYC > 252)
		RJoyYC = 252;
		if (LJoyXC > 252)
		LJoyXC = 252;

		// sprintf(serial_string, "X: %d, Y: %d, X2: %d \n", RJoyXC, RJoyYC, LJoyXC);
		// serial0_print_string(serial_string);  // print the received bytes to the USB serial to make 
		//serial0_print_string("string \r");
		
		//sending section
		if(current_ms-last_send_ms >= 100) //sending rate controlled here one message every 100ms (10Hz)
		{

			sendDataByte1 = RJoyXC;		// Send Right Joystick Horizontal - Robot's movement
			if (sendDataByte1>252) 		//Causes byte 1 to wrap back to 0 when exceeding 253 
			sendDataByte1 = 0;
			
			sendDataByte2 = RJoyYC;		// Send Right Joystick Vertical - Robot's movement
			if (sendDataByte2>252)
			sendDataByte2 = 0;

			sendDataByte3 = LJoyXC;		// Send Left Joystick Horizontal - Camera's Servo
			if (sendDataByte3>253)
			sendDataByte3 = 0;

			sendDataByte4 = button_state;		// State of button 
			if (sendDataByte4>253)
			sendDataByte4 = 0;

			last_send_ms = current_ms;
			serial2_write_byte(0xFF); 		//send start byte = 255
			serial2_write_byte(sendDataByte1); 	//send first data byte: must be scaled to the range 0-253
			serial2_write_byte(sendDataByte2); 	//send second parameter: must be scaled to the range 0-253
			serial2_write_byte(sendDataByte3); 	//send first data byte: must be scaled to the range 0-253
			serial2_write_byte(sendDataByte4); 	//send second parameter: must be scaled to the range 0-253
			serial2_write_byte(0xFE); 		//send stop byte = 254
			//serial0_print_string("Controller sent stop byte\n");
		}

		//if a new byte has been received
		if(new_message_received_flag) 
		{
			R_dist = (int)(496.72 / dataByte1) + 3.0296;
			F_dist = (int)(1748.9/ dataByte2) + 3.2296;
			L_dist = (int)(667.15 / dataByte3) + 0.1011;


			//sprintf(serial_string, "CONTROLLER received: 1:%4d, 2:%4d , 3:%4d , 4:%4d \n", dataByte1, dataByte2, dataByte3, dataByte4);
			//sprintf(serial_string, "X: %u, Y: %u \n", RJoyXC, RJoyYC);
			//serial0_print_string(serial_string);  // print the received bytes to the USB serial to make sure the right messages are received

			lcd_goto(0x0);
			sprintf(lcd_string, "F: %2ucm,", F_dist);
			lcd_puts(lcd_string);
			if (button_state)
			{
				lcd_goto(0x0);
				sprintf(lcd_string, "F: %2ucm  AUTO", F_dist);
				lcd_puts(lcd_string);
				lcd_goto(0x40);
				sprintf(lcd_string, "L:%2ucm, R:%2ucm", L_dist, R_dist);
				//serial0_print_string(lcd_string);
				lcd_puts(lcd_string);
			}
			else
			{
				lcd_clrscr();
				lcd_goto(0x0);
				sprintf(lcd_string, "F: %2ucm", F_dist);
				lcd_puts(lcd_string);
				lcd_goto(0x40);
				sprintf(lcd_string, "L:%2ucm, R:%2ucm", L_dist, R_dist);
				//serial0_print_string(lcd_string);
				lcd_puts(lcd_string);
			}
			// lcd_goto(0x40);
			// sprintf(lcd_string, "L:%2ucm, R:%2ucm", L_dist, R_dist);
			// //serial0_print_string(lcd_string);
			// lcd_puts(lcd_string);

			new_message_received_flag=false;	// set the flag back to false
		}
	}
	return(1);
} //end main


ISR(USART2_RX_vect)  // ISR executed whenever a new byte is available in the serial buffer
{
	static uint8_t recvByte1=0, recvByte2=0, recvByte3=0, recvByte4=0;		// data bytes received
	static uint8_t serial_fsm_state=0;									// used in the serial receive ISR
	uint8_t	serial_byte_in = UDR2; //move serial byte into variable
	//char s_string[32] = {0};
	switch(serial_fsm_state) //switch by the current state
	{
		case 0:
		//do nothing, if check after switch case will find start byte and set serial_fsm_state to 1
		break;
		case 1: //waiting for first parameter
		recvByte1 = serial_byte_in;
		serial_fsm_state++;
		//sprintf(s_string, "state : %u",serial_fsm_state);
		//serial0_print_string(s_string);
		break;
		case 2: //waiting for second parameter
		recvByte2 = serial_byte_in;
		serial_fsm_state++;
		//sprintf(s_string, "state : %u",serial_fsm_state);
		//serial0_print_string(s_string);
		break;
		case 3: //waiting for second parameter
		recvByte3 = serial_byte_in;
		serial_fsm_state++;
		//sprintf(s_string, "state : %u",serial_fsm_state);
		//serial0_print_string(s_string);
		break;
		case 4: //waiting for second parameter
		recvByte4 = serial_byte_in;
		serial_fsm_state++;
		//sprintf(s_string, "state : %u",serial_fsm_state);
		//serial0_print_string(s_string);
		break;
		case 5: //waiting for stop byte
		if(serial_byte_in == 0xFE) //stop byte
		{
			// now that the stop byte has been received, set a flag so that the
			// main loop can execute the results of the message
			dataByte1 = recvByte1;		//Right Sensor Value
			dataByte2 = recvByte2;		//Front Sensor Value
			dataByte3 = recvByte3;		//Left Sensor Value
			dataByte4 = recvByte4;
			
			new_message_received_flag=true;
			//serial0_print_string("Stop byte received from Robot\n");
		}
		// if the stop byte is not received, there is an error, so no commands are implemented
		serial_fsm_state = 0; //do nothing next time except check for start byte (below)
		break;


	}
	if(serial_byte_in == 0xFF) //if start byte is received, we go back to expecting the first data byte
	{
		serial_fsm_state=1;
	}
}
// Button Debounce Interrupt Vector
ISR(INT2_vect)		
{
	//Interrupt code
	uint32_t currentTime = milliseconds_now();    // milisnow() returns current time which is uint32 stored in "currenttime" var.
	static uint32_t previousTime = 0;     //static so that it remembers and not reset every time you call the func.
	if ((currentTime - previousTime) > DEBOUNCE_PERIOD)
	{
		button_state ^= true;
		previousTime = currentTime;
		// lcd_clrscr();
		// lcd_home();
	}	
}

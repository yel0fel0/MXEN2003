//include this .c file's header file
#include "Controller.h"

//static function prototypes, functions only called in this file


//file scope variables
static char serial_string[200] = {0};
volatile uint8_t dataByte1=0, dataByte2=0, dataByte3=0, dataByte4=0;		// data bytes received
volatile bool new_message_received_flag=false;

///Sensor Values into Distance
static uint8_t R_dist = 0;
static uint8_t F_dist = 0;
static uint8_t L_dist = 0;

int main(void)
{
	// initialisation
	serial0_init(); 	// terminal communication with PC
	serial2_init();		// microcontroller communication to/from another Arduino
	// or loopback communication to same Arduino

	adc_init(); //initialse ADC
	_delay_ms(20); //it's a good idea to wait a bit after your init section
	lcd_init(); //initialise ldc and read the hd44780 settings files and update all the reg so it knows what to use.
	_delay_ms(20);
	milliseconds_init();

	//Var declare
	

	uint8_t sendDataByte1=0, sendDataByte2=0, sendDataByte3=0, sendDataByte4=0;		// data bytes sent
	
	uint32_t current_ms=0, last_send_ms=0;			// used for timing the serial send
	
	//UCSRB = (1<<RXEN)|(1<<TXEN
	UCSR2B |= (1 << RXCIE2); // Enable the USART Receive Complete interrupt (USART_RXC)
	DDRA = 0;

	sei();
	

	while(1)
	{
		current_ms = milliseconds_now();
		char lcd_string[32] = {0};
		//uint16_t sense = 5*((dataByte1*100)/1023);
		//uint16_t dist = 2213*10 / (sense*10-18*10);
		uint8_t RJoyXC = (adc_read(1)>>2);		//Read in analog voltage from joystick
		uint8_t RJoyYC = (adc_read(0)>>2);		//Read in analog voltage from joystick
		uint8_t LJoyXC = (adc_read(PD2)>>2);		//Read in analog voltage from joystick

		if (RJoyXC > 252)
		RJoyXC = 252;
		if (RJoyYC > 252)
		RJoyYC = 252;
		if (LJoyXC > 252)
		LJoyXC = 252;

		sprintf(serial_string, "X: %d, Y: %d \n", RJoyXC, RJoyYC);
		serial0_print_string(serial_string);  // print the received bytes to the USB serial to make 

		//sending section
		if(current_ms-last_send_ms >= 100) //sending rate controlled here one message every 100ms (10Hz)
		{

			sendDataByte1 = RJoyXC;
			if (sendDataByte1>252) //Causes byte 1 to wrap back to 0 when exceeding 253
			sendDataByte1 = 0;
			
			sendDataByte2 = RJoyYC;
			if (sendDataByte2>252)
			sendDataByte2 = 0;

			sendDataByte3 = LJoyXC;
			if (sendDataByte3>253)
			sendDataByte3 = 0;
			
			if (sendDataByte4>253)
			sendDataByte4 = 0;
			//You should replace the above data byte code with your own definitions
			//or calculations of what should be sent in the bytes
			
			// you can add additional bytes to send in the message,
			//but make sure the receiving code is expecting the right number of bytes

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
			// now that a full message has been received, we can process the whole message
			// the code in this section will implement the result of your message
			//sprintf(serial_string, "CONTROLLER received: 1:%4d, 2:%4d , 3:%4d , 4:%4d \n", dataByte1, dataByte2, dataByte3, dataByte4);
			R_dist = (int)(1747.2 / dataByte1) - 5.8174;
			F_dist = (int)(663.96/ dataByte2) - 0.5384;
			L_dist = (int)(1747.2 / dataByte3) - 5.8174;

			//sprintf(serial_string, "X: %u, Y: %u \n", RJoyXC, RJoyYC);
			//serial0_print_string(serial_string);  // print the received bytes to the USB serial to make sure the right messages are received

			lcd_goto(0x05);
			sprintf(lcd_string, "F: %2u  cm", F_dist);
			lcd_puts(lcd_string);

			lcd_goto(0x40);
			sprintf(lcd_string, "L: %2u cm  R: %2u cm", L_dist, R_dist);
			//serial0_print_string(lcd_string);
			lcd_puts(lcd_string);

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

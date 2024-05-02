//include this .c file's header file
#include "Robot.h"

//static function prototypes, functions only called in this file
//  ROBOT
// transmit Sensor and receive servo

/************************************************************************
Definitions
************************************************************************/
#define R_PIN 	PF0
#define F_PIN 	PF1
#define L_PIN 	PF2

//file scope variables
static char serial_string[200] = {0};
volatile uint8_t dataByte1=0, dataByte2=0, dataByte3=0, dataByte4=0;		// data bytes received
volatile bool new_message_received_flag=false;
static int16_t lm = 0;
static int16_t rm = 0;

///Sensor Values
static uint16_t R_val = 0;
static uint16_t F_val = 0;
static uint16_t L_val = 0;

static uint16_t R_dist = 0;
static uint16_t F_dist = 0;
static uint16_t L_dist = 0;

int main(void)
{
	// initialisation
 	adc_init(); //initialse ADC
	_delay_ms(20); //it's a good idea to wait a bit after your init section

	serial0_init(); 	// terminal communication with PC
	serial2_init();		// microcontroller communication to/from another Arduino

    milliseconds_init();
	// or loopback communication to same Arduino
	
    //Variable Declarations
	uint8_t sendDataByte1=0, sendDataByte2=0, sendDataByte3=0, sendDataByte4=0;		// data bytes sent
	uint32_t current_ms=0, last_send_ms=0;			// used for timing the serial send

	UCSR2B |= (1 << RXCIE2); // Enable the USART Receive Complete interrupt (USART_RXC) //Meaning every byte that is receiced RX interrupt vector will trigger



	///
	TCCR1A = (1<<COM1A1);
	TCCR1B = (1<<WGM13)|(1<<CS11); 
	ICR1 = 20000; 


	//PWM Req: 100 Hz
	TCCR3A = (1<<COM3A1)|(1<<COM3B1);		//Set CTC on OC3A and OC3B
	TCCR3B = (1<<WGM33)|(1<<CS31);			//MODE 8 prescaler 8
	ICR3 = 10000;                    	   // TOP Value for 100 Hz
	DDRE |= (1<<PE3)|(1<<PE4);          //PE5 - OC3A | PE4-OC3B  | PB5-OC1A as output   
	DDRB |= (1<<PB5);
	DDRA |= (1<<DDA0)|(1<<DDA1)|(1<<DDA2)|(1<<DDA3); //put A0-A3 into low impedance output mode

	//
	uint16_t max = 2400;
	uint16_t min = 600;
	double slope = 1.0 * (min - max) / (1023 - 0);

	sei();
	
	while(1)
	{
		current_ms = milliseconds_now();
		R_val = adc_read(R_PIN);
		F_val = adc_read(F_PIN);
		L_val = adc_read(L_PIN);

		R_dist = (2687.3 / R_val) - 0.5384;
		F_dist = (7071.7 / F_val) - 5.817;
		L_dist = (2687.3 / L_val) - 0.5384;

		//_delay_ms(1000);
		//sprintf(serial_string, "    F:%u \nR:%u   L:%u \n", F_dist, R_dist, L_dist);
		// serial0_print_string(serial_string);		


		//sending section
		if(current_ms-last_send_ms >= 100)//sending rate controlled here one message every 100ms (10Hz)
		{

			sendDataByte1 = (R_val>8);
			if (sendDataByte1>253) //Causes byte 1 to wrap back to 0 when exceeding 253
			sendDataByte1 = 0;
			
			sendDataByte1 = (F_val>8);			
			if (sendDataByte2>253)
			sendDataByte2 = 0;
			
			sendDataByte1 = (L_val>8);
			if (sendDataByte3>253)
			sendDataByte3 = 0;

			if (sendDataByte4>253)
			sendDataByte4 = 0;
			//You should replace the above data byte code with your own definitions
			//or calculations of what should be sent in the bytes
			
			last_send_ms = current_ms;
			serial2_write_byte(0xFF); 		//send start byte = 255
			serial2_write_byte(sendDataByte1); 	//send first data byte: must be scaled to the range 0-253
			serial2_write_byte(sendDataByte2);	//send second parameter: must be scaled to the range 0-253
			serial2_write_byte(sendDataByte3); 	//send first data byte: must be scaled to the range 0-253
			serial2_write_byte(sendDataByte4); 	//send second parameter: must be scaled to the range 0-253
			serial2_write_byte(0xFE); 		//send stop byte = 254
			//serial0_print_string("Stop byte sent by Robot\n");
		}

		//if a new byte has been received
		if(new_message_received_flag) 
		{
			//_delay_ms(1000);
			uint8_t RJoyXR = dataByte1; 
			uint8_t RJoyYR = dataByte2;
			uint8_t LJoyXR = dataByte3;

			sprintf(serial_string, "1--RJoyX: %u, RJoyY: %u \t", RJoyXR, RJoyYR);
			serial0_print_string(serial_string);

			lm = (-RJoyYR + 126) + (RJoyXR - 126) ;
			rm = (-RJoyYR + 126) - (RJoyXR - 126);
			if ((abs(lm - 0) > 5) && (abs(rm - 0) > 5))
			{
				lm = lm;
				rm = rm;
			}
			else
			{
				lm = 0;
				rm = 0;
			}
			sprintf(serial_string, "2--rc: %d, fc: %d \t", (RJoyXR - 126), (-RJoyYR + 126));
			serial0_print_string(serial_string);

			sprintf(serial_string, "3--lm: %d, rm: %d \n", lm, rm);
			serial0_print_string(serial_string);

			OCR3A = (int32_t)abs(lm)*10000/126; //lm speed from magnitude of lm
			OCR3B = (int32_t)abs(rm)*10000/126; //lm speed from magnitude of rm
			OCR1A = 2400 + (slope * LJoyXR);

			sprintf(serial_string, "OCR3A: %d, OCR3B: %d \n", OCR3A, OCR3B);
			serial0_print_string(serial_string);
			if(rm>=0) //if lm is positive
			{
			//set direction forwards
			PORTA &= ~(1<<PA0);
			PORTA |= (1<<PA1);
			serial0_print_string("Right Forward\t");
			}
			else
			{
			//set direction reverse
			PORTA |= (1<<PA0);
			PORTA &= ~(1<<PA1);

			serial0_print_string("Right Backward\n");
			}

			if(lm>=0) //if rm is positive
			{
			//set direction forwards
			PORTA |= (1<<PA2);
			PORTA &= ~(1<<PA3);
			serial0_print_string("Left Forward\t");
			}
			else
			{
			//set direction reverse
			PORTA &= ~(1<<PA2);
			PORTA |= (1<<PA3);
			serial0_print_string("Left Backward\n");
			}

			//sprintf(serial_string, "ROBOT received: 1:%4d, 2:%4d , 3:%4d , 4:%4d \n", dataByte1, dataByte2, dataByte3, dataByte4);
			//serial0_print_string(serial_string);  // print the received bytes to the USB serial to make sure the right messages are received
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

	
	switch(serial_fsm_state) //switch by the current state
	{
		case 0:
		//do nothing, if check after switch case will find start byte and set serial_fsm_state to 1
		break;
		case 1: //waiting for first parameter 
		recvByte1 = serial_byte_in;
		serial_fsm_state++;
		break;
		case 2: //waiting for second parameter # CONTROL SERVO
		recvByte2 = serial_byte_in;
		//OCR1A = 2400 + (slope * recvByte2);
		serial_fsm_state++;
		break;
		case 3: //waiting for second parameter
		recvByte3 = serial_byte_in;
		serial_fsm_state++;
		break;
		case 4: //waiting for second parameter
		recvByte4 = serial_byte_in;
		serial_fsm_state++;
		break;
		case 5: //waiting for stop byte
		if(serial_byte_in == 0xFE) //stop byte
		{
			// now that the stop byte has been received, set a flag so that the
			// main loop can execute the results of the message
			dataByte1 = recvByte1;
			dataByte2 = recvByte2;
			dataByte3 = recvByte3;
			dataByte4 = recvByte4;

			new_message_received_flag=true;
			//serial0_print_string("Stop byte received from Controller\n");
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

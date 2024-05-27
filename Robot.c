//include this .c file's header file
#include "Robot.h"




//static function prototypes, functions only called in this file
//  ROBOT
// transmit Sensor and receive servo

//	Definitions
//Battery monitor pin and Led pin
#define BATT_PIN PF3
#define LED_PIN PB7
#define BATT_LIMIT 930
//Sensor pin Right, Front, Left
#define R_PIN 	PF2
#define F_PIN 	PF1
#define L_PIN 	PF0
//Left motor Positive and Negative
#define LM_POS 	PL3
#define LM_NEG 	PL1
//Right motor Positive and Negative
#define RM_POS 	PC1
#define RM_NEG 	PC3
//Autonomous Definition
#define front_lim	350
#define side_lim	230
#define motor_comp 0.8


//	File scope variables
static char serial_string[200] = {0};
volatile uint8_t dataByte1=0, dataByte2=0, dataByte3=0, dataByte4=0;		//data bytes received
volatile bool new_message_received_flag=false;


//	Variable Declaration

/************************************************************************
Reset microseconds timer
    Reset counter register and corresponding variable.
Input: None
Output: None
************************************************************************/
void milliseconds_reset(void)
{
    uint8_t oldSREG = SREG;         //variable oldSREG stores Status Register, so that interupts can work properly
    TCNT1 = 0;                      //reset microsecond counter
    milliseconds = 0;               //reset microsecond variable
    SREG = oldSREG;
}

/************************************************************************
Return smoothed value (Exponential Filter)
    Reset smoothed value of IR reading.
Input: curr_val, new_val		uint16_t		current value, new value read in
Output: v						uint16_t		smoothed valued
************************************************************************/
uint16_t Exp_filter( uint16_t curr_val, uint16_t new_val)
{

	uint16_t v;
	v = (0.8 * curr_val) + (0.2 * new_val);
	// sprintf(serial_string, "curr %u -- new %u -- v %u \t", curr_val, new_val, v);
	// serial0_print_string(serial_string);
	return v;
}

/************************************************************************
Return distance of right sensor
    Take right sensor reading and return calculated distance in cm
Input: R_data		uint16_t		right sensor reading
Output: r			uint16_t		distance in cm
************************************************************************/
uint16_t R_cm( uint16_t R_data)
{

	uint16_t r;
	// r = (2635.2 / R_data) + 0.4262;
	r = (2010.1 / R_data) + 4.0296;
	// sprintf(serial_string, "curr %u -- new %u -- v %u \t", curr_val, new_val, v);
	// serial0_print_string(serial_string);
	return r;
}

/************************************************************************
Return distance of left sensor
    Take left sensor reading and return calculated distance in cm
Input: L_data		uint16_t		left sensor reading
Output: l			uint16_t		distance in cm
************************************************************************/
uint16_t L_cm( uint16_t L_data)
{

	uint16_t l;
	l = (2635.2 / L_data) + 0.4262;
	// sprintf(serial_string, "curr %u -- new %u -- v %u \t", curr_val, new_val, v);
	// serial0_print_string(serial_string);
	return l;
}

/************************************************************************
Return distance of front sensor
    Take front sensor reading and return calculated distance in cm
Input: F_data		uint16_t		front sensor reading
Output: f			uint16_t		distance in cm
************************************************************************/
uint16_t F_cm( uint16_t F_data)
{

	uint16_t f;
	f = (7078.7 / F_data) - 3.2296;
	// sprintf(serial_string, "curr %u -- new %u -- v %u \t", curr_val, new_val, v);
	// serial0_print_string(serial_string);
	return f;
}

/************************************************************************
Return the error or difference between sensor reading and target
Input: PIN
Output: uint16_t
************************************************************************/
int16_t sensor_error(uint16_t curr_val)
{
	int16_t error;
	error = curr_val - side_lim;
	return error;
}

/************************************************************************
Return the sum of error
    Add current error onto the sum of error
Input: none
Output: sum_error		uint16_t		
************************************************************************/
int16_t sum_error(int16_t curr_error)
{
	static int16_t sum_error;
	sum_error += curr_error;
	// sprintf(serial_string, "Sum E: %i, Curr E: %i \n", sum_error, curr_error);
	// serial0_print_string(serial_string);
	if (sum_error > 60)
	{
		return sum_error = 60;
	}
	else if (sum_error < -60)
	{
		return sum_error = -60;
	}
	else
	{
		return sum_error;
	}
}

/************************************************************************
Return the difference between current error and previous error
Input: prev_error		uint16_t
Output: dif_error		uint16_t		
************************************************************************/
int16_t diff_error(uint16_t prev_error, uint16_t current_error)
{
	int16_t dif_error;
	dif_error = current_error - prev_error;
	sprintf(serial_string, "prev: %i, curr: %i, diff: %i \t\t", prev_error, current_error, dif_error);
	serial0_print_string(serial_string);
	return dif_error;
}



int main(void)
{
	//Initialisation
 	adc_init(); //initialse ADC
	_delay_ms(20); //it's a good idea to wait a bit after your init section

	serial0_init(); 	//terminal communication with PC
	serial2_init();		//microcontroller communication to/from another Arduino

    milliseconds_init();	//set up timer in library to count in miliseconds.
	//or loopback communication to same Arduino
	
    //Variable Declarations
	uint8_t sendDataByte1=0, sendDataByte2=0, sendDataByte3=0, sendDataByte4=0;		//data bytes sent
	uint32_t current_ms=0, last_send_ms=0;			//used for timing the serial send
	uint8_t RJoyXR; 		//Right Joy Stick Horizontal
	uint8_t RJoyYR; 		//Right Joy Stick Vertical
	uint8_t LJoyXR; 		//Left Joy Stick Horizontal

	static int16_t lm = 0;
	static int16_t rm = 0;

	uint16_t R_val = 0;
	uint16_t L_val = 0;
	uint16_t F_val = 0;
	
	static uint16_t R_curr;
	static uint16_t L_curr;
	static uint16_t F_curr;

	uint16_t R_dist = 0;
	uint16_t L_dist = 0;
	uint16_t F_dist = 0;

	int16_t c_error;
	int16_t d_error;
	int16_t PID;
	// uint16_t error = 0;

	double Kp = 0.253;
	double Ki = 0.00;
	double Kd = 0.5;

	volatile static bool self;



	//USART COMMUNICAION
	UCSR2B |= (1 << RXCIE2); 	//Enable the USART Receive Complete interrupt (USART_RXC) in register n B
								//Meaning every byte that is receiced RX interrupt vector will trigger
	//SERVO PWM generation
	TCCR1A = (1<<COM1A1);		//Set compare output mode, Set on down count, Clear on up count on pin OC1A 
	TCCR1B = (1<<WGM13)|(1<<CS11); //Set mode 8 PWM, Phase and Frequency Correct with a Prescaler of 8
	ICR1 = 20000; 				//Top value to interrup at frequency of 50hz
	DDRB |= (1<<PB5)|(1<<LED_PIN);			//Enable as Output: Pin PB5(OC1A) PWM for Servo, LED_PIN for Led of battery monitor

	//MOTOR PWM generation
	TCCR3A = (1<<COM3A1)|(1<<COM3B1);		//Set compare output mode, Set on down count, Clear on up count on pin OC1A 
	TCCR3B = (1<<WGM33)|(1<<CS31);			//Set mode 8 PWM, Phase and Frequency Correct with a Prescaler of 8
	ICR3 = 10000;                    	    //Top value to interrup at frequency of 100hz
	
	DDRE |= (1<<PE3)|(1<<PE4);          	//Enable as Output: Pin PE3(OC3A) and PE4(OC3B) for Left and Right Motor
	DDRL |= (1<<LM_POS)|(1<<LM_NEG);		//Enable as Output: Pin PL3 and PL1 for direction of Left Motor
	DDRC |= (1<<RM_POS)|(1<<RM_NEG);		//Enable as Output: Pin PC1 
	DDRL |= (1<<PD7);
	//Range mapping gradient for servo motor 


	sei();

	while(1)
	{
		sprintf(serial_string, "adc: %u", adc_read(L_PIN));
		serial0_print_string(serial_string);
		// OCR3A = ((int32_t)abs(lm)*10000/126);
		// OCR3B = ((int32_t)abs(rm)*10000/126) - motor_comp;
		current_ms = milliseconds_now();		//milisnow() returns current time which is uint32 stored in "current_ms" var.
		//Filtering sensors analog voltage signal
		R_val = adc_read(R_PIN);			//Read in from pin and assign as a new reading.			
		L_val = adc_read(L_PIN);
		F_val = adc_read(F_PIN);

		R_curr = Exp_filter(R_curr, adc_read(R_PIN));
		L_curr = Exp_filter(L_curr, adc_read(L_PIN));
		F_curr = Exp_filter(F_curr, adc_read(F_PIN));

		//Distance conversion from analogue voltage in to cm
		R_dist = R_cm(R_curr);
		L_dist = L_cm(L_curr);
		F_dist = F_cm(F_curr);


		//SENSOR VAL PRINT
		// sprintf(serial_string, "L adc %u ---- L exp %u \n", L_val, L_curr);
		// serial0_print_string(serial_string);
		// sprintf(serial_string, "%u",(L_curr));
		// serial0_print_string(serial_string);
		// serial0_write_byte(L_dist);

		//Battery Monitor
			//sprintf(serial_string, "Battery Voltage: %f ADC: %u \t",adc_read(BATT_PIN)*5/1024, adc_read(BATT_PIN));
			//serial0_print_string(serial_string);
		if (adc_read(BATT_PIN)< BATT_LIMIT)		//To turn of LED when voltage reading below 4.5v or batter voltage is below 7V.
		{
			PORTB |= (1<<LED_PIN);
			//serial0_print_string("LED ON \n");
		}
		if (adc_read(BATT_PIN)> BATT_LIMIT)
		{
			PORTB &= ~(1<<LED_PIN);
			//serial0_print_string("LED OFF \n");
		}


		//SEND - communication
		if(current_ms-last_send_ms >= 100)		//sending rate controlled here one message every 100ms (10Hz)
		{
			sendDataByte1 = (R_val); 		//Bit shift right by 8
			if (sendDataByte1>253)		 //Causes byte 1 to wrap back to 0 when exceeding 253
			sendDataByte1 = 0;
			
			sendDataByte2 = (F_val);			//Bit shift right by 8		
			if (sendDataByte2>253)
			sendDataByte2 = 0;
			
			sendDataByte3 = (L_val); 		//Bit shift right by 8
			if (sendDataByte3>253)
			sendDataByte3 = 0;
			
			// sendDataByte4 = self;
			if (sendDataByte4>253)
			sendDataByte4 = 0;

			last_send_ms = current_ms;
			serial2_write_byte(0xFF); 		//send start byte = 255
			serial2_write_byte(sendDataByte1); 	//send first data byte: must be scaled to the range 0-253
			serial2_write_byte(sendDataByte2);	//send second parameter: must be scaled to the range 0-253
			serial2_write_byte(sendDataByte3); 	//send third data byte: must be scaled to the range 0-253
			serial2_write_byte(sendDataByte4); 	//send fourth parameter: must be scaled to the range 0-253
			serial2_write_byte(0xFE); 		//send stop byte = 254
			//serial0_print_string("Stop byte sent by Robot\n");
		}

		if(new_message_received_flag)// if(new_message_received_flag) 
		{
			serial0_print_string("msg received\t");
			sprintf(serial_string, "L: %u. F: %u R: %u\n", adc_read(L_PIN), adc_read(F_PIN), adc_read(R_PIN));
			serial0_print_string(serial_string);
			RJoyXR = dataByte1; 
			RJoyYR = dataByte2;
			LJoyXR = dataByte3;
			self = dataByte4;

			double slope = 1.0 * (720 - 1520 ) / (252 - 126);
			OCR1A = 1520 + (slope * (LJoyXR - 126));
			if (!self)			//if self is false -> robot is in manual mode
			{
				serial0_print_string("self is false\t");
				//Convert Joystick value 0~252 to -126~126 
				lm = (-RJoyYR + 126) + (RJoyXR - 126);		
				rm = (-RJoyYR + 126) - (RJoyXR - 126);
				
				OCR3A = (int32_t)abs(lm)*10000/126; //lm speed from magnitude of lm
				OCR3B = (int32_t)abs(rm)*10000/126; //lm speed from magnitude of rm
				OCR1A = 1520 + (slope * (LJoyXR - 126));	//PMW for servo control
				if(lm>=0) //if lm is positive,
				{
					//set LEFT motor direction forwards
					PORTL |= (1<<LM_POS);
					PORTL &= ~(1<<LM_NEG);

				}
				else
				{
					//set LEFT motor direction reverse
					PORTL |= (1<<LM_NEG);
					PORTL &= ~(1<<LM_POS);
				}

				if(rm>=0) //if rm is positive
				{
					//set RIGHT motor direction forwards
					PORTC |= (1<<RM_POS);
					PORTC &= ~(1<<RM_NEG);
				}
				else
				{
					//set RIGHT motor direction reverse
					PORTC |= (1<<RM_NEG);
					PORTC &= ~(1<<RM_POS);
				}
			}
			if(self)			//if self is true -> robot is in autonomous mode
			{
				// serial0_print_string("self is true\t");
				if (adc_read(F_PIN) < front_lim)		//if there is no wall ahead
				{

					if (adc_read(R_PIN) > adc_read(L_PIN))
					{
						static int16_t prevR_error;
						R_curr = Exp_filter(R_curr, adc_read(R_PIN));
						L_curr = Exp_filter(L_curr, adc_read(L_PIN));
						F_curr = Exp_filter(F_curr, adc_read(F_PIN));
						c_error = sensor_error(R_curr);
						d_error = diff_error(prevR_error, c_error);
						prevR_error = c_error;
						int16_t s_error = sum_error(c_error);
						PID = (Kp * c_error) + (Ki * s_error) +(Kd * d_error);
						lm = 90 - PID;
						rm = 90 + PID;
					}
					else if (adc_read(L_PIN) > adc_read(R_PIN))
					{
						// sprintf(serial_string, "\tLEFT_adc: %u, C_error: %i, D_error: %i, S_error: %i, lm: %i, rm: %i, OCR3A: %u, OCR3B: %u, PID: %i , \n", L_curr, c_error, d_error, s_error, lm, rm, OCR3A, OCR3B, PID);
						// serial0_print_string(serial_string);
						static int16_t prevL_error;
						R_curr = Exp_filter(R_curr, adc_read(R_PIN));
						L_curr = Exp_filter(L_curr, adc_read(L_PIN));
						F_curr = Exp_filter(F_curr, adc_read(F_PIN));
						c_error = sensor_error(L_curr);
						d_error = diff_error(prevL_error, c_error);
						prevL_error = c_error;
						int16_t s_error = sum_error(c_error);
						PID = (Kp * c_error) + (Ki * s_error) +(Kd * d_error);
						lm = 90 + PID;
						rm = 90 - PID;
					}

					//MOTOR DRIVE
					// lm = 90 + PID;
					// rm = 90 - PID;
					if(lm>=0) //if lm is positive,
					{
						//set LEFT motor direction forwards
						PORTL |= (1<<LM_POS);
						PORTL &= ~(1<<LM_NEG);

					}
					else
					{
						//set LEFT motor direction reverse
						PORTL |= (1<<LM_NEG);
						PORTL &= ~(1<<LM_POS);
					}

					if(rm>=0) //if rm is positive
					{
						//set RIGHT motor direction forwards
						PORTC |= (1<<RM_POS);
						PORTC &= ~(1<<RM_NEG);
					}
					else
					{
						//set RIGHT motor direction reverse
						PORTC |= (1<<RM_NEG);
						PORTC &= ~(1<<RM_POS);
					}
					OCR3A = ((int32_t)abs(lm)*10000/126);
					OCR3B = ((int32_t)abs(rm)*10000/126);
					// sprintf(serial_string, "Left PWM: %u, Right PWM: %u, lm: %i, rm: %i\n", OCR3A, OCR3B, lm, rm);
					// serial0_print_string(serial_string);
				}
				else// DEAD END
				{	
					OCR3A = 0;
					OCR3B = 0;
					_delay_ms(500);
					if(adc_read(L_PIN) > side_lim/2)		//Left is blocked -> turn RIGHT
					{
						milliseconds_reset();
						while(adc_read(F_PIN) > 300)
						{
							serial0_print_string("Dead end \t");
							serial0_print_string("turn right 90 \t");
							sprintf(serial_string, "R: %u. F: %u \n", adc_read(R_PIN), adc_read(F_PIN));
							serial0_print_string(serial_string);
							PORTL |= (1<<LM_POS);
							PORTL &= ~(1<<LM_NEG);
							PORTC |= (1<<RM_NEG);
							PORTC &= ~(1<<RM_POS);
							OCR3A = 7000 * 1.1;		//Left motor speed
							OCR3B = 7000;		//Right motor speed
							_delay_ms(350);
							OCR3A = 0;
							OCR3B = 0;
							if (adc_read(L_PIN) > side_lim && milliseconds_now() > 800)
							{
								break;
							}
						}
					}
					else if(adc_read(R_PIN) > side_lim/2)		//Right is blocked -> turn LEFT
					{
						milliseconds_reset();
						while(adc_read(F_PIN) > 300)
						{
							serial0_print_string("Dead end \t");
							serial0_print_string("turn right 90 \t");
							sprintf(serial_string, "R: %u. F: %u \n", adc_read(R_PIN), adc_read(F_PIN));
							serial0_print_string(serial_string);
							PORTL |= (1<<LM_NEG);
							PORTL &= ~(1<<LM_POS);
							PORTC |= (1<<RM_POS);
							PORTC &= ~(1<<RM_NEG);
							OCR3A = 7000;		//Left motor speed
							OCR3B = 7000 * 1.1;		//Right motor speed
							_delay_ms(350);
							OCR3A = 0;
							OCR3B = 0;
							// PORTB |= (1<<LED_PIN);
							if (adc_read(R_PIN) > side_lim && milliseconds_now() > 800)
							{
								break;
							}
						}
					}					
				}				
			}
			new_message_received_flag=false;		//set the flag back to false now that the all packets received
		}
	}
	return(1);
} //end main

//RECEIVE - communication
ISR(USART2_RX_vect)  //ISR executed whenever a new byte is available in the serial buffer
{
	static uint8_t recvByte1=0, recvByte2=0, recvByte3=0, recvByte4=0;		//data bytes received
	static uint8_t serial_fsm_state=0;									//used in the serial receive ISR
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
			//now that the stop byte has been received, set a flag so that the
			//main loop can execute the results of the message
			dataByte1 = recvByte1;
			dataByte2 = recvByte2;
			dataByte3 = recvByte3;
			dataByte4 = recvByte4;

			new_message_received_flag=true;
			//serial0_print_string("Stop byte received from Controller\n");
		}
		//if the stop byte is not received, there is an error, so no commands are implemented
		serial_fsm_state = 0; //do nothing next time except check for start byte (below)
		break;
	}
	if(serial_byte_in == 0xFF) //if start byte is received, we go back to expecting the first data byte
	{
		serial_fsm_state=1;
	}
}
/*! \file util.h
    \brief Utility functions for the Atmel platform.
	
	For an overview of how timer based interrupts work, see
	page 111 and 133-137 of the Atmel Mega128 User Guide
*/

/// Blocks for a specified number of milliseconds
void wait_ms(unsigned int time_val);

/// Start timer2
void timer2_start(char unit);

/// Stop timer2
void timer2_stop();

/// Initializes the Analog-to-Digital converter.
/**
* ADMUX: REFS=11, ADLAR= 0, MUX don't care; ADCSRA: ADEN=1, ADFR=0, ADIE=0, ADPS=111, others don't care.
*/
void ADC_init();

/// Reads digitally converted values. Written by Dalton.
/** 
* Sets ADSC bit of ADCSRA, enabling ADC. 
* @return ADC The conversion temp
*/
unsigned int read_ADC();

/// Converts the ADC value to centimeters. Written by Dalton and improved upon by Omar.
/**
* A function that takes the average of five ADC samples and calculates the corresponding centimeter value as a result.
*/
int read_IR_distance();

/// Initializes the ping sensor.
/** 
* TCCR1A: WGM1[1:0]=00; TCCR1B: Noise canceller ON, falling edge is trigger, prescaler of 64; TIMSK: Enable TICIE1
*/
void ping_timer_init();

/// Pulses the SONAR sensor.
void send_pulse();

/// Converts values from the SONAR to centimeters. Written by Dalton and improved upon by Omar.
/** 
* A function that takes calculates the centimeter value from the sensor using the following equation: (delta/(Frequency/pre-scaler))*(speed of sound/2)
*/
float read_PING_distance();

/// Initializes the servo.
/** 
* A function that initializes the ISR timer for the servo, setting the TOP value using the following equation: pulse period in cycles; (clock_frequency/(prescaler * 1000)) * pulse period. Servo degrees is calculated using the following equation: ((clock_frequency/(prescaler * 1000)) * pulse_time_in_ms) * calibration_value
*/
void servo_timer_init();

/// Moves the servo to the by a specified degree. Written by Omar.
/** 
* A function that moves the servo by converting the degree value to a pulse width. The function does not allow the servo to move beyond 180 degrees.
* @param degrees value to be converted to a pulse width
*/
void move_servo(volatile float* degrees);

/// Readies the USART for communication.
/** 
* @param ubrr constitutes: clock rate / (system bit / speed) / (baud rate - 1). See page 362 of User Guide for register summary.
*/
void USART_Init(unsigned int ubrr);

/// Transmits over serial buffer per character. Enabled by USART_Init.
/**
* Waits for empty transmit buffer before it sends a character.
* @param data character to be sent
*/
void USART_Transmit(unsigned char data);

/// Waits for data to be received and returns the sent character. Enabled by USART_Init.
/**
* Waits for data to be received before getting and returning the received data from the buffer.
*/
unsigned char USART_Receive(void);

/// Calls USART_Transmit for each character in the array. Written by Omar.
/**
* @param message array of character to be looped through and sent over USART until a null character is found.
*/
void send_message(char *message);


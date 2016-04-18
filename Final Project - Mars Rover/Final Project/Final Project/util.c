/**
 * util.c: utility functions for the Atmel platform
 * 
 * For an overview of how timer based interrupts work, see
 * page 111 and 133-137 of the Atmel Mega128 User Guide
 *
 * @author Zhao Zhang & Chad Nelson
 * @date 06/26/2012
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <math.h>
#include "util.h"

// Global used for interrupt driven delay functions
volatile unsigned int timer2_tick;
void timer2_start(char unit);
void timer2_stop();


/// Blocks for a specified number of milliseconds
void wait_ms(unsigned int time_val) {
	//Seting OC value for time requested
	OCR2=250; 				//Clock is 16 MHz. At a prescaler of 64, 250 timer ticks = 1ms.
	timer2_tick=0;
	timer2_start(0);

	//Waiting for time
	while(timer2_tick < time_val);

	timer2_stop();
}


// Start timer2
void timer2_start(char unit) {
	timer2_tick=0;
	if ( unit == 0 ) { 		//unit = 0 is for slow 
        TCCR2=0b00001011;	//WGM:CTC, COM:OC2 disconnected,pre_scaler = 64
        TIMSK|=0b10000000;	//Enabling O.C. Interrupt for Timer2
	}
	if (unit == 1) { 		//unit = 1 is for fast
        TCCR2=0b00001001;	//WGM:CTC, COM:OC2 disconnected,pre_scaler = 1
        TIMSK|=0b10000000;	//Enabling O.C. Interrupt for Timer2
	}
	sei();
}


// Stop timer2
void timer2_stop() {
	TIMSK&=~0b10000000;		//Disabling O.C. Interrupt for Timer2
	TCCR2&=0b01111111;		//Clearing O.C. settings
}


// Interrupt handler (runs every 1 ms)
ISR (TIMER2_COMP_vect) {
	timer2_tick++;
}

/************************************************************************/
/* IR Program                                                           */
/************************************************************************/
void ADC_init()
{
	// REFS=11, ADLAR= 0, MUX don't care
	ADMUX |= (3<<REFS0) | (2<<MUX0); //(REFS1) | _BV(REFS0);
	
	// ADEN=1, ADFR=0, ADIE=0, ADPS=111, others don't care.
	//See page 246 of user guide
	ADCSRA |= (1<<ADEN) | (7<<ADPS0);
}

unsigned int read_ADC()
{
	ADMUX |= (PF2 & 0x1F);
	//Sets ADSC bit of ADCSRA, enabling ADC
	ADCSRA |= (1<<ADSC);
	//Waits for conversion to be done.
	while(ADCSRA & (1<<ADFR)){}
	//Sets conversion to temp var.
	return ADC;
}

int read_IR_distance() {
	int sum = 0;
	for (int i = 0; i < 5; i++)
	sum += read_ADC();
	
	int quantVal = sum/5;
	
	return (pow(quantVal,-1.171) * 31427);
}
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

/************************************************************************/
/* PING Program                                                         */
/************************************************************************/

volatile unsigned rise = 0;
volatile unsigned fall = 0;
volatile unsigned signal = 0;

void ping_timer_init()
{
	TCCR1A = 0x00;		// WGM1[1:0]=00
	TCCR1B = 0b11000011; // Noise canceller ON, falling edge is trigger, prescaler of 64
	TIMSK = 1 << TICIE1; // Enable TICIE1
}

void send_pulse()
{
	TIMSK &= ~0b00100100;
	DDRD |= 0x10;
	PORTD |= 0x10;
	wait_ms(1);
	PORTD &= 0xEF;
	DDRD &= 0xEF;
	
	TIFR |= (1 << ICF1); // Clear flag
	
	TIMSK |= 0b00100000;
}

/* Input Capture Event for Timer1 */
ISR(TIMER1_CAPT_vect)
{
	if (rise == 0)
	{
		rise = ICR1;
		
	}
	else if (fall == 0)
	{
		fall = ICR1;
		signal = fall - rise;
		
		rise = 0;
		fall = 0;
	}
	
	// Rising or falling?
	TCCR1B ^= 0b01000000;
}

float read_PING_distance() {
	return ((signal/(16000000.0/64.0))*(34300.0/2.0)); // (delta/(Frequency/pre-scaler))*(speed of sound/2)
}
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

/************************************************************************/
/* Servo Program                                                        */			//currently calibrated for robot 3
/************************************************************************/
#define TOP (16000000/(8 * 1000)) * 21.5                 // pulse period in cycles; (clock_frequency/(prescaler * 1000)) * pulse period
#define ZERO ((16000000/(8 * 1000)) * 1) * 0.45         // 1 ms pulse - clockwise far end; ((16000000/(8 * 1000)) * 1ms) * .5075 (0.4 for bot 3; 0.45 for bot 17) calibration
#define NINTY ((16000000/(8 * 1000)) * 1.5) * 0.8833      // 1.5 ms pulse - center position; ((16000000/(8 * 1000)) * 1.5ms) * .9397 (0.816666 for bot 3; 0.8833 for bot 17) calibration
#define ONE_EIGHTY ((16000000/(8 * 1000)) * 2) * 1.0875  // 2 ms pulse - counterclockwise far end; ((16000000/(8 * 1000)) * 2ms) * 1.1362 (1.0375 for bot 3; 1.0875 for bot 17) calibration

void servo_timer_init() {
	TCCR3A = 0b00100011; //set COM and WGM
	TCCR3B = 0b00011010; //set WGM and CS
	OCR3A = TOP;
	
	DDRE |= _BV(4); // Set PE4 as output
	
	OCR3B = NINTY; // Initialize servo to center
	wait_ms(500); // Wait for Servo to get into position
}

void move_servo(volatile float* degrees)
{
	if (*degrees <= 180 && *degrees >= 0) // Prevent servo from going out of range
	OCR3B = (ZERO + (*degrees/180) * (ONE_EIGHTY - ZERO)) - 1; // Convert values to degrees and store as pulse width
	
	// Prevent angle from going out of bounds
	if (*degrees > 180)
	*degrees = 180;
	else if (*degrees < 0)
	*degrees = 0;
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

/************************************************************************/
/* USART Program (Bluetooth)                                            */
/************************************************************************/
/************************************************************************/
/* Readies the USART for communication                                  */
/* ubrr constitutes: clock rate / (system bit / speed) / (baud rate - 1)*/
/* See page 362 of User Guide for register summary                      */
/************************************************************************/
void USART_Init(unsigned int ubrr)
{
	/* Set baud rate. Put the upper part of the baud number here (bits 8 to 11) */
	UBRR0H = (unsigned char) (ubrr >> 8);
	
	/*Put the remaining part of the baud number here*/
	UBRR0L = (unsigned char) ubrr;
	
	UCSR0A = (1 << U2X0); /* Steps Double Speed Asynchronous mode of communication */
	UCSR0B = (1 << RXEN0) | (1 << TXEN); /* Enable receiver and transmitter */

	UCSR0C = (1 << USBS0) | (3 << UCSZ00); /* Set frame format: 8data, 2stop bit */
}
/************************************************************************/
/* Transmits over serial buffer per character.                          */
/* Enabled by USART_Init                                                */
/************************************************************************/
void USART_Transmit(unsigned char data)
{
	/* Wait for empty transmit buffer */
	while (!(UCSR0A & (1 << UDRE0))) ;
	UDR0 = data; /* Put data into buffer, sends the data */
}
/************************************************************************/
/* Waits for data to be received and returns the sent character.        */
/* Enabled by USART_Init                                                */
/************************************************************************/
unsigned char USART_Receive(void)
{
	/* Wait for data to be received */
	while (!(UCSR0A & (1 << RXC0))) ;
	
	return UDR0; /* Get and return received data from buffer */
}
/************************************************************************/
/* Calls USART_Transmit for each character in the array                 */
/************************************************************************/
void send_message(char *message)
{
	for (int i = 0; message[i] != '\0'; i++)
		USART_Transmit(message[i]);
}
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/*
 * GccApplication16.c
 *
 * Created: 2021/10/21 23:58:50
 * Author : 大能耐
 */ 

#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)

#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "uart.h"
int range = 0;
int stepComp = 0;
int step = 0;
int mode = 0;
int counter = 0;
int start = 0;
int end = 0;
int period = 0;
int ovfl = 0;
char string1[25];

void Initialize(){
	cli();
	//GPIO pins
	DDRB &= ~(1<<DDB0);
	DDRB |= (1<<DDB1);
	DDRD &= ~(1<<DDD6);		//input pin 6 button
	DDRD |= (1<<DDD5);		//buzzer
	PORTD |= (1<<PORTD5);	//pull up pd5
	PORTD |= (1<<PORTD6);
	PORTB &= ~(1<<PORTB1);
	
	//timer prescale of 8
	TCCR1B |= (1<<CS11);
	
	//normal mode
	TCCR1A &= ~(1<<WGM10);
	TCCR1A &= ~(1<<WGM11);
	TCCR1B &= ~(1<<WGM12);
	TCCR1B &= ~(1<<WGM13);
	
	//*******ADC****************
	PRR &= ~(1<<PRADC);
	//set VREF = AVCC
	ADMUX |= (1<<REFS0);
	ADMUX &= ~(1<<REFS1);
	
	//set ADC clock div by 128
	ADCSRA |= (1<<ADPS0);
	ADCSRA |= (1<<ADPS1);
	ADCSRA |= (1<<ADPS2);
	
	//select channel 0
	ADMUX &= ~(1<<MUX0);
	ADMUX &= ~(1<<MUX1);
	ADMUX &= ~(1<<MUX2);
	ADMUX &= ~(1<<MUX3);
	
	//set free running
	ADCSRA |= (1<<ADATE);	//auto trigger
	ADCSRB &= ~(1<<ADTS0);
	ADCSRB &= ~(1<<ADTS1);
	ADCSRB &= ~(1<<ADTS2);
	
	//disable digital input buffer on ADC pin
	DIDR0 = (1<<ADC0D);
	
	//enable ADC
	ADCSRA |= (1<<ADEN);
	
	//enable ADC interrupt
	ADCSRA |= (1<<ADIE);
	
	//start conversion
	ADCSRA |= (1<<ADSC);
	//********************************
	
	//********timer 0********************
	//timer prescale of 64
	TCCR0B |= (1<<CS01);
	TCCR0B |= (1<<CS00);
	
	//PWM mode, phase correct mode 5, top OCRA
	TCCR0A |= (1<<WGM00);
	TCCR0A &= ~(1<<WGM01);
	TCCR0B |= (1<<WGM02);
	
	//output mode, toggle  compare match
	TCCR0A |= (1<<COM0A0);
	TCCR0A &= ~(1<<COM0A1);
	
	TCCR0A &= ~(1<<COM0B0);
	TCCR0A |= (1<<COM0B1);
	
	OCR0A = 59;
	
	//***************************************
	
	//enable PCINT22 pin change interrupt
	PCICR |= (1<<PCIF2);
	//enable trigger for PCINT 
	PCMSK2 |=(1<<PCINT22);
	
	TCCR1B |= (1<<ICES1);		//looking for rising edge
	TIFR1 |= (1<<ICF1);			//clear interrupt flag
	TIFR1 |= (1<<TOV1);			//overflow flag
	TIMSK1 |= (1<<ICIE1);		//enable input capture interrupt
	TIMSK1 |= (1<<OCF1A);		//enable overflow interrupt
	OCR1A = 20;
	
	//timer output compare match interrupt enable
	TIMSK1 |= (1<<OCIE1A);
	//timer output compare match flag
	TIFR1 |= (1<<OCF1A);
	sei();
}

void discreteFreq(){
	if( range >2 && range <= 33){
		OCR0A = 118;
	}else if(range >33 && range <= 64){
		OCR0A =106;
	}else if(range >64 && range <= 95){
		OCR0A =94;
	}else if(range >95 && range <= 126){
		OCR0A =88;
	}else if(range >126 && range <= 157){
		OCR0A =78;
	}else if(range >157 && range <= 188){
		OCR0A =70;
	}else if(range >188 && range <= 219){
		OCR0A =62;
	}else if(range >219 && range <= 249){
		OCR0A =58;
	}
}

void printRange(){
	sprintf(string1,"The distance is: %u cm %u \n", range, mode);
	UART_putstring(string1);
	_delay_ms(50);
}

void findRangeAndFreq(){
	period = 65535*ovfl + end - start;
	range = (period/2)/58;				//cm
	ovfl = 0;							//reset overflow
	if(mode == 0){
		OCR0A = 0.243*range + 57;
	}else{
		discreteFreq();
	}
	
}

ISR(PCINT2_vect){
	counter++;
	if(counter == 2){
		if(mode == 0){
			//continue
			mode = 1;
		}else{
			//Discrete
			mode =0;
		}
		counter = 0;
	}
}

//when TCNT1 = OCR1A
ISR(TIMER1_COMPA_vect){
	if(stepComp == 0){
		PORTB |= (1<<PORTB1);
		OCR1A += 20;
		stepComp++;
	}else{
		PORTB &= ~(1<<PORTB1);
		stepComp = 0;
	}
}

ISR(TIMER1_CAPT_vect){
	if(step == 0){
		start = ICR1;
		step++;
	}else{
		end = ICR1;
		step = 0;
		findRangeAndFreq();
		printRange();
	}
	TCCR1B ^= (1<<ICES1);	//capture the opposite edge
}

ISR(TIMER1_OVF_vect){
	ovfl++;
}

//changing duty cycle
ISR(ADC_vect){
	if(ADC >0 && ADC <=213){
		OCR0B = 0.05* OCR0A;
	}else if(ADC>213 && ADC <=287){
		OCR0B = 0.1* OCR0A;
	}else if(ADC>287 && ADC <=361){
		OCR0B = 0.2* OCR0A;
	}else if(ADC>361 && ADC <=435){
		OCR0B = 0.25* OCR0A;
	}else if(ADC>435 && ADC <=509){
		OCR0B = 0.3* OCR0A;
	}else if(ADC>585 && ADC <=658){
		OCR0B = 0.35* OCR0A;
	}else if(ADC>658 && ADC <=733){
		OCR0B = 0.4* OCR0A;
	}else if(ADC>733 && ADC <=808){
		OCR0B = 0.45* OCR0A;
	}else if(ADC>808 && ADC <=882){
		OCR0B = 0.5* OCR0A;
	}
}

int main(void)
{
	UART_init(BAUD_PRESCALER);
	Initialize();
	while (1)
	{
	}
}

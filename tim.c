#include<avr/io.h>
#include<avr/interrupt.h>
#include <util/delay.h>

void sendc(char c) {
    while(!(UCSR0A & _BV(UDRE0)));
        UDR0 = c;
}

#define BAUD 9600
#define BRC ((F_CPU/16/BAUD) - 1)
uint8_t i = 0;

#define TVAL (((1<<16)-1)-16953)

typedef struct {
	float t;
	uint8_t mode;
} instr;

const instr path[] = {{2, 2}, {0.5, 0}, {1.5, 2}, {0.5, 0}, {1, 2}, {0.5, 0}};
uint8_t ix = -1;
instr curr = path[0];

void next_instr() {
	ix = (ix+1)%(sizeof(path)/sizeof(path[0]));
	curr = path[ix];
	TCNT1 = -16953.25*curr.t;   // for 1 sec at 16 MHz
	if (curr.mode) {
		PORTB |= 2;
	} else {
		PORTB &= ~2;
	}
}

ISR (TIMER1_OVF_vect)    // Timer1 ISR
{	
	next_instr();
}





int main()
{
	DDRB = 2;
	PORTB = 3;

	// usart
    UBRR0H = BRC >> 8;
    UBRR0L = BRC;
    UCSR0B = _BV(TXEN0) | _BV(RXEN0);
    UCSR0C = 0x6;
	
	//TCNT1 = TVAL;   // for 1 sec at 16 MHz	
	next_instr();

	TCCR1A = 0x00;
	TCCR1B = (1<<CS10) | (1<<CS12);;  // Timer mode with 1024 prescler
	TIMSK1 = (1 << TOIE1) ;   // Enable timer1 overflow interrupt(TOIE1)
	sei();        // Enable global interrupts by setting global interrupt enable bit in SREG

	
	
	for(;;) {
		_delay_ms(500);
		PORTB ^= 1;
	}
}
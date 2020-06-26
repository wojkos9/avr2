#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#define BAUD 9600
#define BRC ((F_CPU/16/BAUD) - 1)

void sendc(char c) {
    while(!(UCSR0A & _BV(UDRE0)));
        UDR0 = c;
}

void main() {
    UBRR0H = BRC >> 8;
    UBRR0L = BRC;
    UCSR0B = _BV(TXEN0) | _BV(RXEN0);
    UCSR0C = 0x6;
    char c = '0';
    for(;;) {
        _delay_ms(500);
        sendc(c++);
    }
}
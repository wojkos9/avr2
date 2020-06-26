#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define MOT PORTD2

uint8_t s = 1;
const uint8_t m = 5, range = 100;
uint8_t motor = 0;

ISR(PCINT1_vect) {
    if (!(PINC & _BV(PINC5))) {
        _delay_ms(50);
        if ((PINC & _BV(PINC5))) goto leave;
        PORTB ^= 1;
        motor ^= 1;
        if (motor)
            s = (s+1)%m;
    }
    leave:
    ;
}


void main() {
    DDRB = 1;
    DDRD = _BV(MOT);

    // button
    PORTC |= 32;
    PCMSK1 |= _BV(PCINT13);
    PCICR |= _BV(PCIE1);
    sei();

    uint8_t i=0;
    for(;;) {
        PORTD |= _BV(MOT);
        _delay_us(1000);
        for(i=s; i; i--) _delay_us(range/m);
        if (motor)
            PORTD &= ~_BV(MOT);
        _delay_us(20000-1000-range);
        for(i=m-s; i; i--) _delay_us(range/m);
    }
}
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define MOT1 PORTD2

uint8_t motor = 0;
const uint8_t m = 20;

uint8_t s1=0, s2 = 0;
const int d1 = 920;
const int range=160;

#define BAUD 9600
#define BRC ((F_CPU/16/BAUD) - 1)

void sendc(char c) {
    while(!(UCSR0A & _BV(UDRE0)));
        UDR0 = c;
}

void sendn(int16_t n) {
    const int maxc = 5;
    char t[maxc];
    uint8_t i = 0;
    if (n<0) {
        sendc('-');
        n = -n;
    }
    do {
        t[maxc-++i]='0'+n%10;
        n /= 10;
    } while (n);
    while (i) {
        sendc(t[maxc- i--]);
    }
}


ISR(PCINT1_vect) {
    if (!(PINC & _BV(PINC5))) {
        _delay_ms(50);
        if ((PINC & _BV(PINC5))) goto leave;
        PORTB = s1 & 7;
        //motor = 1-motor;
        //if (motor)
        s1 = (s1+1)%(m+1);
        sendn(s1-m/2);
        sendc('\r');
        sendc('\n');
    }
    leave:
    ;
}


void motor_control(uint8_t id, uint8_t s) {
    uint8_t i = 0;
    PORTD |= _BV(id);
    _delay_us(d1);
    for(i=s; i; i--) _delay_us(range/m);
    PORTD &= ~_BV(id);
    _delay_us(10000-d1-range);
    for(i=m-s; i; i--) _delay_us(range/m);
}

void loop() {
    if (motor) {
        motor_control(MOT1, s1);
        //motor_control(MOT2, s2);
    }
}


void main() {
    // 3 leds
    DDRB = 7;
    //PORTB = 1;

    motor = 1;

    DDRD = _BV(MOT1);
    s1 = 0;

    // usart
    UBRR0H = BRC >> 8;
    UBRR0L = BRC;
    UCSR0B = _BV(TXEN0);
    UCSR0C = 0x6;

    // button
    PORTC |= 32;
    PCMSK1 |= _BV(PCINT13);
    PCICR |= _BV(PCIE1);

    sei();
    
    for(;;) {
        loop();
    }

}
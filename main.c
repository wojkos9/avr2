#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define MOT1 PORTB1
#define MOT2 PORTB2

uint8_t motor = 0;
const uint8_t m = 20;

uint8_t s1=0, s2 = 0;
const int d1 = 920;
const int range=160;

void sendc(char c) {
    while(!(UCSR0A & _BV(UDRE0)));
        UDR0 = c;
}

#define BAUD 9600
#define BRC ((F_CPU/16/BAUD) - 1)


void motor_control(uint8_t id, uint8_t s) {
    uint8_t i = 0;
    PORTB |= _BV(id);
    _delay_us(d1);
    for(i=s; i; i--) _delay_us(range/m);
    PORTB &= ~_BV(id);
    _delay_us(10000-d1-range);
    for(i=m-s; i; i--) _delay_us(range/m);
}

void loop() {
    if (motor) {
        motor_control(MOT1, s1);
        motor_control(MOT2, 20-s2);
    }
    
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

uint8_t sens = 0;
uint8_t sid = 0;
#define NSENS 3



ISR(ADC_vect)
{
    uint16_t r = ADC;
    
    if (r > 800) {
        sens |= 1<<sid;
    } else {
        sens &= ~(1<<sid);
    }
    sid = (sid+1)%NSENS;
    // pins 3,4,5
    // sendc('s');
    // sendn((_BV(REFS0) | (sid+3)));
    // sendc(' ');
    // sendn(r);
    // sendc('\r');
    // sendc('\n');
    ADMUX = _BV(REFS0) | (sid+3);
    ADCSRA |= _BV(ADSC);
}

void main() {

    motor = 1;

    DDRB = _BV(MOT1) | _BV(MOT2);
    s1 = 10;
    s2 = 10;

    // usart
    UBRR0H = BRC >> 8;
    UBRR0L = BRC;
    UCSR0B = _BV(TXEN0) | _BV(RXEN0);
    UCSR0C = 0x6;

    //adc
    ADMUX |= _BV(REFS0) | (sid+3);
    DIDR0 |= _BV(ADC3D) | _BV(ADC4D) | _BV(ADC5D);
    ADCSRA |= _BV(ADEN) | _BV(ADIE) | _BV(ADPS0) | _BV(ADPS1) | _BV(ADPS2);
    ADCSRA |= _BV(ADSC);

    sei();
    
    for(;;) {
        loop();

        /*
        111,010 full full
        110, 011 1/2 full 2/1 pol
        100 001 1/2 full 2/1 stop
        000 nic
        101
        */

        switch (sens & 7) {
            case 0:
            case 5:
                s1 = s2 = 10;
                break;
            case 7:
            case 2:
                s1 = 20;
                s2 = 20;

                break;

            case 3:
                s1 = 20;
                s2 = 15;
                break;

            case 6:
                s1 = 15;
                s2 = 20;
                break;

            case 1:
                s1 = 20;
                s2 = 10;
                break;

            case 4:
                s1 = 10;
                s2 = 20;
                break;
        }
        // sendc('0'+((sens & 4)?1:0));
        // sendc('0'+((sens & 2)?1:0));
        // sendc('0'+((sens & 1)?1:0));
        // sendc(' ');
        // sendn(s1);
        // sendc(' ');
        // sendn(s2);
        // sendc('\r');
        // sendc('\n');


        //sendc(sens+'0');
    }

}
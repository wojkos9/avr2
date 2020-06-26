#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define MOT1 PORTD2
#define MOT2 PORTD3

uint8_t motor = 0;
const uint8_t m = 10;

uint8_t s1=0, s2 = 0;
const int d1 = 1000;
    const int range=80;

void sendc(char c) {
    while(!(UCSR0A & _BV(UDRE0)));
        UDR0 = c;
}

#define BAUD 9600
#define BRC ((F_CPU/16/BAUD) - 1)

ISR(PCINT1_vect) {
    if (!(PINC & _BV(PINC5))) {
        _delay_ms(50);
        if ((PINC & _BV(PINC5))) goto leave;
        PORTB ^= 1;
        motor = 1-motor;
        //if (motor)
        s1 = (s1+1)%m;
        s2 = (s2+1)%m;
        sendc(s1+'0');
        sendc('0');
        sendc('%');
        sendc(' ');
        sendc(s2+'0');
        sendc('0');
        sendc('%');
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
        motor_control(MOT2, s2);
    }
    
}

uint8_t sens = 0;
uint8_t sid = 0;
#define NSENS 3



ISR(ADC_vect)
{
    uint16_t r = ADC;
    if (r > 512) {
        sens |= 1<<sid;
    } else {
        sens &= ~(1<<sid);
    }
    sid = (sid+1)%NSENS;
    ADMUX = _BV(REFS0) | sid;
    ADCSRA |= _BV(ADSC);
}

void main() {
    // 3 leds
    DDRB = 7;
    //PORTB = 1;

    motor = 1;

    DDRD = _BV(MOT1) | _BV(MOT2);
    s1 = 5;
    s2 = 5;

    // usart
    UBRR0H = BRC >> 8;
    UBRR0L = BRC;
    UCSR0B = _BV(TXEN0) | _BV(RXEN0);
    UCSR0C = 0x6;

    // button
    PORTC |= 32;
    PCMSK1 |= _BV(PCINT13);
    PCICR |= _BV(PCIE1);

    //adc
    ADMUX |= _BV(REFS0);
    DIDR0 |= _BV(ADC0D) | _BV(ADC1D) | _BV(ADC2D);
    ADCSRA |= _BV(ADEN) | _BV(ADIE) | _BV(ADPS0) | _BV(ADPS1) | _BV(ADPS2);
    ADCSRA |= _BV(ADSC);

    sei();
    
    for(;;) {
        loop();
        //sendc(sens+'0');
        PORTB = sens;

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
                s1 = s2 = 0;
                break;
            case 7:
            case 2:
                s1 = 0;
                s2 = 10;

                break;

            case 3:
                s1 = 10;
                s2 = 5;
                break;

            case 6:
                s1 = 5;
                s2 = 10;
                break;

            case 1:
                s1 = 10;
                s2 = 0;
                break;

            case 4:
                s1 = 0;
                s2 = 10;
                break;
        }
        sendc('0'+s1);
        sendc('0'+s2);
        sendc('\r');
        sendc('\n');


        //sendc(sens+'0');
    }

}
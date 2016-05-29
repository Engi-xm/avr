#include <avr/power.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "dht11.h"

void init_timer1(void);

int main(void) {
	// setup
	init_timer1();
	DDRB |= 0xff;
	uint8_t temp, hmdty;
	_delay_ms(1000);

	// loop
	while(1) {
		read_dht11(&temp, &hmdty);
		PORTB = temp;
		_delay_ms(2000);
		PORTB = hmdty;
		_delay_ms(1000);
	}

	return 0;
}

void init_timer1(void) {
	TCCR1B |= (1 << CS10);
}

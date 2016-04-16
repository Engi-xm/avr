#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <util/delay.h>
#include "dht11.h"

#define DHT_TIM '1'
#define DHT_PORT 'B'
#define DHT_PIN 1

static inline void initTimer1(void);

int main() {
	// setup
	initTimer1();
	DDRA |= 0x3f;
	_delay_ms(2000);
	uint8_t temp = 0;
	uint8_t hmdty = 0;

	// loop
	while(1) {
		readDHT11(&temp, &hmdty);
		PORTA = temp;
		_delay_ms(1000);
		PORTA = hmdty;
		_delay_ms(2000);

	}

	return 0;
}

static inline void initTimer1() {
	TCCR1B |= (1 << CS10); // /1 prescaler
}

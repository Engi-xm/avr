#include <avr/power.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/setbaud.h>
#include "usart_mod.h"
#include "dht11.h"

#define BUTTON0 0x01
#define BUTTON1 0x02
#define DHT_TIM '1'
#define DHT_PORT 'D'
#define DHT_PIN 7

static void init_tim1(void);
static uint8_t *itoa(uint8_t number);

int main(void) {
	// setup
	DDRB = 0xff;
	init_usart();
	init_tim1();
	// uint8_t temp, hmdty;
	uint8_t data1[2] = "11";
	uint8_t data2[2] = "55";
	uint8_t prefix1[] = "n0.val=";
	uint8_t prefix2[] = "j0.val=70";
	uint8_t suffix[3] = {0xff, 0xff, 0xff};
	uint8_t ref[5] = "ref 0";
	uint8_t *p;
	p = itoa(5);
	sei();
	_delay_ms(1500);

	// loop
	while(1) {
		// readDHT11(&temp, &hmdty);
		usart_send(p, 2);
		_delay_ms(1000);
	}

	return 0;
}

static void init_tim1(void) {
	TCCR1B = (1 << CS10); // /1 prescaler
}

static uint8_t *itoa(uint8_t number) {
	static uint8_t buf[3];
	uint8_t *p = buf + 2; // point to ending
	while(number != 0) {
		*--p = '0' + (number % 10); // increment backwards
		number /= 10;
	}
	return p;
}

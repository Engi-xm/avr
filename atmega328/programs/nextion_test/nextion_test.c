#include <avr/power.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/setbaud.h>
#include "usart_mod.h"

#define BUTTON0 0x01
#define BUTTON1 0x02

int main(void) {
	// setup
	uint8_t data[7];
	DDRB = 0xff;
	init_usart();
	sei();

	// loop
	while(1) {
		usart_read(7);
		while(usart_rx_busy);
		for(uint8_t x = 0; x < 7; x++) {
			data[x] = usart_rx_buffer[x];
		}
		switch(data[2]) {
			case BUTTON1 :
				PORTB ^= (1 << 1);
				break;
			case BUTTON0 :
				PORTB ^= (1 << 0);
				break;
			default :
				PORTB = 0xf0;
				break;
		}
		_delay_ms(200);
	}

	return 0;
}

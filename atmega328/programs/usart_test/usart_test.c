#include <avr/power.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/setbaud.h>
#include "usart.h"

int main(void) {
	// setup
	DDRB = 0xff;
	uint8_t symb[1] = {0x61};
	init_usart();
	sei();

	// loop
	while(1) {
		// usart_read(1);
		// while(usart_rx_busy);
		usart_send(symb, 1);
		_delay_ms(500);
		// PORTB = usart_rx_buffer[0];
	}

	return 0;
}

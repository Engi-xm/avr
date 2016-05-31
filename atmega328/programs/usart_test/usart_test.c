#include <avr/power.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/setbaud.h>
#include "usart.h"

int main(void) {
	// setup
	DDRB = 0xff;
	uint8_t symb[3] = {0x61, 0x62, '\0'};
	uint8_t* p_symb;
	init_usart();
	sei();

	// loop
	while(1) {
		// p_symb = usart_read(1);
		// while(usart_rx_busy);
		// PORTB = *p_symb;
		usart_send(symb);
		_delay_ms(500);
	}

	return 0;
}

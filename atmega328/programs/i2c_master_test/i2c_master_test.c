#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <util/delay.h>
#include "i2c_master.h"

static inline void init_int0(void);

#define ARRAY_LENGTH(array) (sizeof(array)/sizeof((array)[0])) // array length macro

#define SLAVE_ADDR 0b1101000
#define REG_ADDR_1 0x00
#define REG_ADDR_2 0x07

ISR(INT0_vect) {
	PORTB ^= (1 << 7);
}

int main() {
	// setup
	// clock_prescale_set(clock_div_1);
	DDRB |= 0xff;
	uint8_t send_data[2];
	init_i2c(50000);
	// init_int0();
	sei(); // FUCKING INTERRUPTS
	send_data[0] = 0b00000000; // TURN THE FUCKING CRYSTAL ON (bit7 == 0)
	send_data[1] = 0b00001001;
	i2c_send(SLAVE_ADDR, REG_ADDR_1, send_data, 2);
	// send_data[0] = 0b00010000; // set out to 1Hz
	// i2c_send(SLAVE_ADDR, REG_ADDR_2, send_data, 1);

	// loop
	while(1) {
		i2c_read(SLAVE_ADDR, REG_ADDR_1, 2);
		_delay_ms(50);
		PORTB = (rx_buffer[1] >> 4) * 10 + (rx_buffer[1] & 0x0f);
		_delay_ms(200);
		// PORTB = (rx_buffer[0] >> 4) * 10 + (rx_buffer[0] & 0x0f);
		// _delay_ms(200);
	}
		return 0;
}

static inline void init_int0(void) {
	EICRA |= (1 << ISC01); // trigger on falling edge
	EIMSK |= (1 << INT0); // enable interrupt
}

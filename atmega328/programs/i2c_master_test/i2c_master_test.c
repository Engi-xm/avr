#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <util/delay.h>
#include "i2c_master.h"

static inline void init_int0(void);

#define SLAVE_ADDR 0b1101000
#define REG_ADDR_1 0x0e
#define REG_ADDR_2 0x00

ISR(INT0_vect) {
	PORTB ^= (1 << 7);
}

int main() {
	// setup
	DDRB |= 0xff;
	uint8_t send_data[2];
	uint8_t* p_read_data;
	init_i2c(50000);
	// init_int0();
	sei(); // FUCKING INTERRUPTS
	send_data[0] = 0b01000110; // TURN THE FUCKING CRYSTAL ON (bit7 == 0)
	send_data[1] = 0x08;
	i2c_send(SLAVE_ADDR, REG_ADDR_1, send_data);

	// loop
	while(1) {
		p_read_data = i2c_read(SLAVE_ADDR, REG_ADDR_2, 1);
		while(i2c_busy);
		PORTB = (*p_read_data >> 4) * 10 + (*p_read_data & 0x0f);
		_delay_ms(300);
	}
		return 0;
}

static inline void init_int0(void) {
	EICRA |= (1 << ISC01); // trigger on falling edge
	EIMSK |= (1 << INT0); // enable interrupt
}

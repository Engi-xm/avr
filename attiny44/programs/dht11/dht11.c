#include "dht11.h"

void readDHT11(uint8_t* temp, uint8_t* hmdty) {
	// initialization
	uint8_t data[5];
	uint8_t arrayIndex = 0;
	uint8_t bitIndex = 7;
	uint8_t startTime;
	uint8_t signalTime;

	// clear data buffer
	for (int i = 0; i < 5; i++) {
		data[i] = 0;
	}

	// send request
	DHT_DDR |= (1 << DHT_PIN);
	DHT_W &= ~(1 << DHT_PIN);
	_delay_ms(18);
	cli();	// turn off interrupts for timing
	DHT_W |= (1 << DHT_PIN);
	_delay_us(40);
	DHT_DDR &= ~(1 << DHT_PIN);
	_delay_us(10);

	// wait for response
	loop_until_bit_is_set(DHT_R, DHT_PIN);
	loop_until_bit_is_clear(DHT_R, DHT_PIN);

	// read 40 bits
	for (int i = 0; i < 40; i++) {
		// wait for signal start
		loop_until_bit_is_set(DHT_R, DHT_PIN);

		// time signal
		startTime = DHT_TIM_CNT;
		loop_until_bit_is_clear(DHT_R, DHT_PIN);
		signalTime = DHT_TIM_CNT - startTime;

		// push high bit if signal longer than 40us
		if (signalTime > 40) {
			data[arrayIndex] |= (1 << bitIndex);
		}

		// increment or reset indexes
		if (bitIndex == 0) {
			bitIndex = 7;
			arrayIndex++;
		} else {
			bitIndex--;
		}
	}

	// re-enable interrupts
	sei();
	
	// if checksum passed, update data
	if (data[4] == (data[0] + data[2])) {
		*temp = data[2];
		*hmdty = data[0];
	}
}

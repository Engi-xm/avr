// include header
#include "i2c_master.h"

// private function prototypes
static inline void i2c_start(void); // send start signal
static inline void i2c_stop(void); // send stop signal
static inline void i2c_send_byte(uint8_t data); // send a byte
static inline void i2c_read_byte_ack(void); // receive a byte with acknowledge
static inline void i2c_read_byte_nack(void); // receive a byte without acknowledge

// structure for rx and tx buffers
typedef struct buf {
	uint8_t slave_addr_buffer; // slave address buffer
	uint8_t addr_buffer; // register address buffer
	uint8_t buffer_length; // length of data buffer
} buffer_struct;

// global scope variables
volatile uint8_t i2c_rx_buffer[1]; // receive data buffer

// private variables
static volatile uint8_t tx_mode = 1; // selection between tx and rx modes
static volatile uint8_t is_busy = 0; // status variable
static volatile uint8_t tx_buffer[1]; // transmit data buffer
static volatile buffer_struct rx; // receive buffer
static volatile buffer_struct tx; // transmit buffer

// i2c interrupt
ISR(TWI_vect) {
	switch(I2C_STATUS) {
		case I2C_START_SENT:
			if(tx_mode) { // if tx mode is selected
				i2c_send_byte(tx.slave_addr_buffer); // send trasmit slave address
			} else {
				i2c_send_byte(rx.slave_addr_buffer); // send receive slave address
			}
			break;
		case I2C_MT_SLAW_ACK:
			i2c_send_byte(tx.addr_buffer); // send register address
			break;
		case I2C_MT_DATA_ACK:
			if(tx.buffer_length > 0) { // if havent reached end of tx buffer
				i2c_send_byte(tx_buffer[--tx.buffer_length]); // send data from tx buffer
			} else { // if reached end of tx buffer
				i2c_stop(); // send stop signal
			}
			break;
		case I2C_MR_SLAR_ACK:
			if(rx.buffer_length > 1) { // if more than one byte needs to be read
				i2c_read_byte_ack(); // read byte with ack
			} else {
				i2c_read_byte_nack(); // read byte with nack
			}
			break;
		case I2C_MR_DATA_ACK:
			i2c_rx_buffer[--rx.buffer_length] = TWDR; // read data
			if(rx.buffer_length > 1) { // if more than one byte needs to be read
				i2c_read_byte_ack(); // read byte with ack
			} else {
				i2c_read_byte_nack(); // read byte with nack
			}
			break;
		case I2C_MR_DATA_NACK:
			i2c_rx_buffer[0] = TWDR; // read data
			i2c_stop(); // send stop signal
			break;
		default:
			i2c_stop(); // send stop signal
			break;
	}
}

static inline void i2c_start(void) {
	TWCR = ((1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (1 << TWIE)); // send start signal
}

static inline void i2c_stop(void) {
	TWCR = ((1 << TWINT) | (1 << TWSTO) | (1 << TWEN) | (1 << TWIE)); // send stop signal
	is_busy = 0; // set to available
}

static inline void i2c_send_byte(uint8_t data) {
	TWDR = data; // load data to data register
	TWCR = ((1 << TWINT) | (1 << TWEN) | (1 << TWIE)); // send byte stored in TWDR
}

static inline void i2c_read_byte_ack(void) {
	TWCR = ((1 << TWINT) | (1 << TWEN) | (1 << TWEA) | (1 << TWIE)); // receive byte, send ack
}

static inline void i2c_read_byte_nack(void) {
	TWCR = ((1 << TWINT) | (1 << TWEN) | (1 << TWIE)); // receive byte, send nack	
}

void init_i2c(uint16_t freq) {
	if(freq > I2C_FREQ_MAX || freq < I2C_FREQ_MIN) { // if outside limits
		freq = I2C_FREQ; // set to default
	}
	TWBR = ((F_CPU / freq) - 16) / 2; // set SCL frequency 
	TWCR |= (1 << TWIE); // enable interrupts
	TWCR |= (1 << TWEN); // turn on i2c module
}

void i2c_send(uint8_t slave_addr, uint8_t addr, uint8_t *data, uint8_t i) {
	while(is_busy); // check if available
	is_busy = 1; // set to busy
	tx_mode = 1; // set transmit mode
	tx.slave_addr_buffer = (uint8_t)(slave_addr << 1); // load slave address to buffer
	tx.addr_buffer = addr; // load address to buffer
	tx.buffer_length = i; // get number of bytes to transmit
	for (uint8_t x = 0; x < tx.buffer_length; x++) { // load data to buffer
		tx_buffer[x] = data[x];
	}
	i2c_start(); // start transmition
}

void i2c_read(uint8_t slave_addr, uint8_t addr, uint8_t i) {
	i2c_send(slave_addr, addr, 0, 0);
	while(is_busy); // check if available
	is_busy = 1; // set to busy
	tx_mode = 0; // set receive mode
	rx.slave_addr_buffer = (uint8_t)((slave_addr << 1) + 1); // load slave address to buffer
	// rx.addr_buffer = addr; // load address to buffer (for book keeping)
	rx.buffer_length = i; // set number of bytes to receive
	for(uint8_t x = 0; x < i; x++) {
		i2c_rx_buffer[x] = 0; // clear receive buffer
	}
	i2c_start(); // start transmition
}

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// outer functions prototypes
inline void init_i2c(uint16_t freq); // initialize i2c bus
void i2c_send(uint8_t slave_addr, uint8_t addr, uint8_t *data, uint8_t i); // send i data bytes to slave
void i2c_read(uint8_t slave_addr, uint8_t addr, uint8_t i); // read i bytes from slave
// inner functions prototypes
static inline void i2c_start(void); // send start signal
static inline void i2c_stop(void); // send stop signal
static inline void i2c_send_byte(uint8_t data); // send a byte
static inline void i2c_read_byte_ack(void); // receive a byte with acknowledge
static inline void i2c_read_byte_nack(void); // receive a byte without acknowledge

static inline void init_int0(void);

// status codes
#define I2C_START_SENT     0x08 // start signal sent
#define I2C_REP_START_SENT 0x10 // repeated start sent
#define I2C_MT_SLAW_ACK    0x18 // slave write sent, ack received
#define I2C_MT_SLAW_NACK   0x20 // slave write sent, nack received
#define I2C_MT_DATA_ACK    0x28 // data sent, ack received
#define I2C_MT_DATA_NACK   0x30 // data sent, nack received
#define I2C_MR_SLAR_ACK    0x40 // slave read sent, ack received
#define I2C_MR_SLAR_NACK   0x48 // slave read sent, nack received
#define I2C_MR_DATA_ACK    0x50 // data received, ack sent
#define I2C_MR_DATA_NACK   0x58 // data received, nack sent
#define I2C_STATUS (TWSR & 0xf8) // status of i2c
// other defines
#ifndef I2C_FREQ
	#define I2C_FREQ 50000 // default to 50kHz
#endif
#ifndef I2C_FREQ_MAX
	#define I2C_FREQ_MAX 100000 // set max freq to 100kHz
#endif
#ifndef I2C_FREQ_MIN
	#define I2C_FREQ_MIN 20000 // set min to 20kHz
#endif
#define ARRAY_LENGTH(array) (sizeof(array)/sizeof((array)[0])) // array length macro

#define SLAVE_ADDR 0b1101000
#define REG_ADDR_1 0x00
#define REG_ADDR_2 0x07

// structure for rx and tx buffers
typedef struct buf {
	uint8_t slave_addr_buffer; // slave address buffer
	uint8_t addr_buffer; // register address buffer
	uint8_t buffer_length; // length of data buffer
} buffer_struct;
// global scope variables
volatile uint8_t rx_buffer[2]; // receive data buffer
// file scope variables
static volatile uint8_t tx_mode = 1; // selection between tx and rx modes
static volatile uint8_t is_busy = 0; // status variable
static volatile uint8_t tx_buffer[2]; // transmit data buffer
static volatile buffer_struct rx; // receive buffer
static volatile buffer_struct tx; // transmit buffer
// static volatile uint8_t dummy = 0;
// static volatile uint8_t inc = 0;

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
			rx.buffer_length--; // decrement to previous byte
			rx_buffer[rx.buffer_length] = TWDR; // read data
			if(rx.buffer_length > 1) { // if more than one byte needs to be read
				i2c_read_byte_ack(); // read byte with ack
			} else {
				i2c_read_byte_nack(); // read byte with nack
			}
			break;
		case I2C_MR_DATA_NACK:
			rx_buffer[0] = TWDR; // read data
			i2c_stop(); // send stop signal
			break;
		default:
			i2c_stop(); // send stop signal
			break;
	}
}

ISR(INT0_vect) {
	PORTB ^= (1 << 7);
}

int main() {
	// setup
	// clock_prescale_set(clock_div_1);
	DDRB |= 0xff;
	uint8_t send_data[2];
	init_i2c(I2C_FREQ);
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
		_delay_ms(1000);
		// PORTB = (rx_buffer[0] >> 4) * 10 + (rx_buffer[0] & 0x0f);
		// _delay_ms(200);
	}
		return 0;
}

static inline void init_int0(void) {
	EICRA |= (1 << ISC01); // trigger on falling edge
	EIMSK |= (1 << INT0); // enable interrupt
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

inline void init_i2c(uint16_t freq) {
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
		rx_buffer[x] = 0; // clear receive buffer
	}
	i2c_start(); // start transmition
}

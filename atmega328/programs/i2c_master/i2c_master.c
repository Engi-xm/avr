#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// outer functions prototypes
inline void initI2c(void); // initialize i2c bus
void i2cSend(uint8_t slaveAddr, uint8_t addr, uint8_t *data, uint8_t i); // send i data bytes to slave
void i2cRead(uint8_t slaveAddr, uint8_t addr, uint8_t i); // read i bytes from slave
// inner functions prototypes
static inline void i2cStart(void); // send start signal
static inline void i2cStop(void); // send stop signal
static inline void i2cSendByte(uint8_t data); // send a byte
static inline void i2cReadByteAck(void); // receive a byte with acknowledge
static inline void i2cReadByteNack(void); // receive a byte without acknowledge

static inline void initInt0(void);

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
#define ARRAY_LENGTH(array) (sizeof(array)/sizeof((array)[0])) // array length macro

#define SLAVE_ADDR 0b1101000
#define REG_ADDR_1 0x00
#define REG_ADDR_2 0x07

// structure for rx and tx buffers
typedef struct buf {
	uint8_t SlaveAddrBuffer; // slave address buffer
	uint8_t AddrBuffer; // register address buffer
	uint8_t BufferLength; // length of data buffer
} bufferStruct;
// global scope variables
volatile uint8_t rxBuffer[1]; // receive data buffer
// file scope variables
static volatile uint8_t txMode = 1; // selection between tx and rx modes
static volatile uint8_t isBusy = 0; // status variable
static volatile uint8_t txBuffer[1]; // transmit data buffer
static volatile bufferStruct rx; // receive buffer
static volatile bufferStruct tx; // transmit buffer
// static volatile uint8_t dummy = 0;
// static volatile uint8_t inc = 0;

ISR(TWI_vect) {
	switch(I2C_STATUS) {
		case I2C_START_SENT:
			if(txMode) { // if tx mode is selected
				i2cSendByte(tx.SlaveAddrBuffer); // send trasmit slave address
			} else {
				i2cSendByte(rx.SlaveAddrBuffer); // send receive slave address
			}
			break;
		case I2C_MT_SLAW_ACK:
			i2cSendByte(tx.AddrBuffer); // send register address
			break;
		case I2C_MT_DATA_ACK:
			if(tx.BufferLength > 0) { // if havent reached end of tx buffer
				i2cSendByte(txBuffer[--tx.BufferLength]); // send data from tx buffer
			} else { // if reached end of tx buffer
				i2cStop(); // send stop signal
			}
			break;
		case I2C_MR_SLAR_ACK:
			if(rx.BufferLength > 1) { // if more than one byte needs to be read
				i2cReadByteAck(); // read byte with ack
			} else {
				i2cReadByteNack(); // read byte with nack
			}
			break;
		case I2C_MR_DATA_ACK:
			rx.BufferLength--; // decrement to previous byte
			rxBuffer[rx.BufferLength] = TWDR; // read data
			if(rx.BufferLength > 1) { // if more than one byte needs to be read
				i2cReadByteAck(); // read byte with ack
			} else {
				i2cReadByteNack(); // read byte with nack
			}
			break;
		case I2C_MR_DATA_NACK:
			rxBuffer[0] = TWDR; // read data
			i2cStop(); // send stop signal
			break;
		default:
			i2cStop(); // send stop signal
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
	uint8_t sendData[1];
	initI2c();
	// initInt0();
	sei(); // FUCKING INTERRUPTS
	sendData[0] = 0b00000000; // TURN THE FUCKING CRYSTAL ON (bit7 == 0)
	i2cSend(SLAVE_ADDR, REG_ADDR_1, sendData, ARRAY_LENGTH(sendData));
	sendData[0] = 0b00010000; // set out to 1Hz
	i2cSend(SLAVE_ADDR, REG_ADDR_2, sendData, ARRAY_LENGTH(sendData));
	// loop
	while(1) {
		i2cRead(SLAVE_ADDR, REG_ADDR_1, 1);
		_delay_ms(500);
		PORTB = (rxBuffer[0] >> 4) * 10 + (rxBuffer[0] & 0x0f);
	}
		return 0;
}

static inline void initInt0(void) {
	EICRA |= (1 << ISC01); // trigger on falling edge
	EIMSK |= (1 << INT0); // enable interrupt
}

static inline void i2cStart(void) {
	TWCR = ((1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (1 << TWIE)); // send start signal
}

static inline void i2cStop(void) {
	TWCR = ((1 << TWINT) | (1 << TWSTO) | (1 << TWEN) | (1 << TWIE)); // send stop signal
	isBusy = 0; // set to available
	// dummy = 0;
}

static inline void i2cSendByte(uint8_t data) {
	TWDR = data; // load data to data register
	TWCR = ((1 << TWINT) | (1 << TWEN) | (1 << TWIE)); // send byte stored in TWDR
}

static inline void i2cReadByteAck(void) {
	TWCR = ((1 << TWINT) | (1 << TWEN) | (1 << TWEA) | (1 << TWIE)); // receive byte, send ack
}

static inline void i2cReadByteNack(void) {
	TWCR = ((1 << TWINT) | (1 << TWEN) | (1 << TWIE)); // receive byte, send nack	
}

inline void initI2c(void) {
	TWBR = ((F_CPU / I2C_FREQ) - 16) / 2; // set SCL frequency 
	TWCR |= (1 << TWIE); // enable interrupts
	TWCR |= (1 << TWEN); // turn on i2c module
}

void i2cSend(uint8_t slaveAddr, uint8_t addr, uint8_t *data, uint8_t i) {
	while(isBusy); // check if available
	isBusy = 1; // set to busy
	txMode = 1; // set transmit mode
	tx.SlaveAddrBuffer = (uint8_t)(slaveAddr << 1); // load slave address to buffer
	tx.AddrBuffer = addr; // load address to buffer
	tx.BufferLength = i; // get number of bytes to transmit
	for (uint8_t x = 0; x < i; x++) { // load data to buffer
		txBuffer[x] = data[x];
	}
	i2cStart(); // start transmition
}

void i2cRead(uint8_t slaveAddr, uint8_t addr, uint8_t i) {
	i2cSend(slaveAddr, addr, 0, 0);
	while(isBusy); // check if available
	isBusy = 1; // set to busy
	txMode = 0; // set receive mode
	rx.SlaveAddrBuffer = (uint8_t)((slaveAddr << 1) + 1); // load slave address to buffer
	// rx.AddrBuffer = addr; // load address to buffer (for book keeping)
	rx.BufferLength = i; // set number of bytes to receive
	for(uint8_t x = 0; x < i; x++) {
		rxBuffer[x] = 0; // clear receive buffer
	}
	i2cStart(); // start transmition
}

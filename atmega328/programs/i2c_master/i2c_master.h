/* Interrupt based I2C library. Use outer functions to use I2C bus.
Function initi2c initializes bus (use with interrupts off)
Function i2c_send sends data bytes to slave
Function i2c_read reads data bytes from slave to global array i2c_rx_buffer */

#include <avr/io.h>
#include <avr/interrupt.h>

// outer functions prototypes
void init_i2c(uint16_t freq); // initialize i2c bus
void i2c_send(uint8_t slave_addr, uint8_t addr, uint8_t *data, uint8_t i); // send i data bytes to slave
void i2c_read(uint8_t slave_addr, uint8_t addr, uint8_t i); // read i bytes from slave

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

// global scope variables
extern volatile uint8_t i2c_rx_buffer[2]; // receive data buffer

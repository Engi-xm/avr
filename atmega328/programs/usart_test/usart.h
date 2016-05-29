/* Interrupt based USART library. Use functions to communicate via
USART bus.
Function init_usart initializes bus (atomic)
Function usart_send sends data over tx from global buffer array
Function usart_read reads data from rx and stores to global buffer array
Library provides gloval flag for waiting for a message*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/setbaud.h>

// outer function prototypes
void init_usart(void); // initialize usart
void usart_send(uint8_t* data, uint8_t i); // send message
uint8_t* usart_read(uint8_t max_length); // read message

// defines
#ifndef USART_RX_BUF_LENGTH
	#define USART_RX_BUF_LENGTH 10 // default to 10 bytes
#endif

// global scope variables
extern volatile uint8_t usart_rx_busy; // rx busy flag

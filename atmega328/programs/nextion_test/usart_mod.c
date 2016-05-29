// include header
#include "usart_mod.h"

// inner function prototypes
static void usart_send_byte(uint8_t data); // send single byte
static uint8_t usart_read_byte(void); // read single byte to buffer
static void usart_stop_tx(void); // stop transmiter
static void usart_stop_rx(void); // stop receiver

// global scope variables
volatile uint8_t usart_rx_buffer[USART_RX_BUF_LENGTH]; // rx buffer
volatile uint8_t usart_tx_buffer[USART_RX_BUF_LENGTH]; // tx buffer
volatile uint8_t usart_rx_busy = 0; // rx busy flag

// private variables
static volatile uint8_t usart_tx_busy = 0; // tx busy flag
static volatile uint8_t usart_rx_buffer_length; // rx buffer length
static volatile uint8_t usart_tx_buffer_length; // tx buffer length

// interrupt handlers
ISR(USART_RX_vect) { // rx complete interrupt
	// TODO: set end character
	static uint8_t i = 0; // static increment variable
	usart_rx_buffer[i] = usart_read_byte(); // read byte to global buffer
	if (++i >= usart_rx_buffer_length) { // if reached max length
		usart_rx_buffer[i] = '\0'; // close buffer
		i = 0; // reset increment
		usart_stop_rx(); // stop rx
	}
}
ISR(USART_UDRE_vect) { // UDR empty interrupt
	static uint8_t i = 0; // static increment variable
	if (i < usart_tx_buffer_length) { // if havent reached end of tx buffer
		usart_send_byte(usart_tx_buffer[i++]); // send next byte
	} else {
		i = 0; // reset increment
		usart_stop_tx(); // stop tx
	}
}

void init_usart(void) {
	// TODO: set variable speed (add input)
	UBRR0L = UBRRH_VALUE; // set baudrate bits
	UBRR0L = UBRRL_VALUE; // pt. 2
	#if USE_2X // set 2X mode if necessary
	  UCSR0A |= (1 << U2X0);
	#else
	  UCSR0A &= ~(1 << U2X0);
	#endif
	UCSR0C |= ((1 << UCSZ01) | (1 << UCSZ00)); // set 8bit mode
}

void usart_send(uint8_t *data, uint8_t i) {
	while(usart_tx_busy); // wait for available
	usart_tx_busy = 1; // set to busy
	usart_tx_buffer_length = i; // set length
	for (uint8_t x = 0; x < usart_tx_buffer_length; x++) { // load data to buffer
		usart_tx_buffer[x] = data[x];
	}
	usart_tx_buffer[usart_tx_buffer_length] = '\0'; // close buffer
	UCSR0B |= ((1 << TXEN0) | (1 << UDRIE0)); // turn on tx and interrupt
}

void usart_read(uint8_t max_length) {
	while(usart_rx_busy); // wait for available
	usart_rx_busy = 1; // set to busy
	usart_rx_buffer_length = max_length; // set max length
	UCSR0B |= ((1 << RXEN0) | (1 << RXCIE0)); // turn on rx and interrupt
}

static void usart_stop_tx(void) {
	UCSR0B &= ~((1 << TXEN0) | (1 << UDRIE0)); // turn off tx and interrupt
	usart_tx_busy = 0; // set to available
}

static void usart_stop_rx(void) {
	UCSR0B &= ~((1 << RXEN0) | (1 << RXCIE0)); // turn off rx and interrupt
	usart_rx_busy = 0; // set to available
}

static void usart_send_byte(uint8_t data) {
	UDR0 = data; // load data byte to buffer
}

static uint8_t usart_read_byte(void) {
	return UDR0; // read data byte from buffer
}

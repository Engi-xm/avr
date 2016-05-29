// include header
#include "usart.h"

// inner function prototypes
static void usart_send_byte(uint8_t data); // send single byte
static uint8_t usart_read_byte(void); // read single byte to buffer
static void usart_stop_tx(void); // stop transmiter
static void usart_stop_rx(void); // stop receiver

// global scope variables
volatile uint8_t usart_rx_busy = 0; // rx busy flag

// private variables
static volatile uint8_t tx_busy = 0; // tx busy flag
static volatile uint8_t rx_buffer[USART_RX_BUF_LENGTH]; // rx buffer
static volatile uint8_t rx_word_len; // word length for rx
static volatile uint8_t* p_tx_data; // pointer to tx data
static volatile uint8_t* p_rx_data; // pointer to rx data

// interrupt handlers
ISR(USART_RX_vect) { // rx complete interrupt
	// TODO: set end character
	static uint8_t i = 0; // static increment variable
	*(p_rx_data + i) = usart_read_byte(); // read byte to buffer
	if (++i >= rx_word_len) { // if reached max length
		i = 0; // reset increment
		usart_stop_rx(); // stop rx
	}
}
ISR(USART_UDRE_vect) { // UDR empty interrupt
	if (*(p_tx_data) == '\0') { // if reached end of tx buffer
		usart_stop_tx(); // stop tx
	}
	usart_send_byte(*(p_tx_data++)); // send byte and increment
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

void usart_send(uint8_t* data) {
	while(tx_busy); // wait for available
	tx_busy = 1; // set to busy
	p_tx_data = data; // set pointer to data
	UCSR0B |= ((1 << TXEN0) | (1 << UDRIE0)); // turn on tx and interrupt
}

uint8_t* usart_read(uint8_t max_length) {
	while(usart_rx_busy); // wait for available
	usart_rx_busy = 1; // set to busy
	p_rx_data = rx_buffer; // set pointer to buffer
	if (max_length > USART_RX_BUF_LENGTH) { // if word longer than buffer
		max_length = USART_RX_BUF_LENGTH; // set it to buffer length
	}
	rx_word_len = max_length; // set max length
	UCSR0B |= ((1 << RXEN0) | (1 << RXCIE0)); // turn on rx and interrupt
	return p_rx_data; // return pointer to data
}

static void usart_stop_tx(void) {
	UCSR0B &= ~((1 << TXEN0) | (1 << UDRIE0)); // turn off tx and interrupt
	tx_busy = 0; // set to available
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

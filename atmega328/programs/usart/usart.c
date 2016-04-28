#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <util/delay.h>
#include <util/setbaud.h>

// function prototypes
void init_usart(void); // initialize usart
void usart_send(uint8_t *data, uint8_t i); // send message
void usart_read(uint8_t max_length); // read message
void usart_send_byte(uint8_t data); // send single byte
uint8_t usart_read_byte(void); // read single byte to buffer
void usart_stop_tx(void); // stop transmiter
void usart_stop_rx(void); // stop receiver

// defines
#define ARRAY_LENGTH(array) (sizeof(array)/sizeof((array)[0])) // array length macro

// global scope variables
volatile uint8_t usart_rx_buffer[1]; // rx buffer
volatile uint8_t usart_tx_buffer[1]; // tx buffer
volatile uint8_t usart_rx_complete = 1; // rx complete flag
volatile uint8_t tx_busy = 0; // tx busy flag
volatile uint8_t rx_busy = 0; // rx busy flag
volatile uint8_t usart_rx_buffer_length;
volatile uint8_t usart_tx_buffer_length;

// interrupt handlers
ISR(USART_RX_vect) { // rx complete interrupt
	static uint8_t i = 0; // static increment variable
	if ((i < usart_rx_buffer_length) && (!usart_rx_complete)) { // if havent reached end
		usart_rx_buffer[i++] = usart_read_byte(); // read byte to global buffer
		if (usart_rx_buffer[i] == '\r') { // if read return char
			usart_rx_buffer[i] = '\0'; // close buffer
			usart_rx_complete = 1; // set return
		}
	} else {
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

// main
int main(void) {
	// setup
	DDRB |= 0xff; 
	uint8_t send_data[2] = {0x61, 0x62}; // send a, b
	init_usart();
	sei();
	
	// loop
	while(1) {
		// usart_read(50);
		// while(rx_busy);
		usart_send(send_data, 1);
		while(tx_busy);
		_delay_ms(500);
	}

	return 0;
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
	// UCSR0B |= ((1 << RXCIE0) | (1 << TXCIE0) | (1 << UDRIE0)); // enable interrupts
	// UCSR0B |= ((1 << RXEN0) | (1 << TXEN0)); // enable rx and tx pins
}

void usart_stop_tx(void) {
	UCSR0B &= ~(1 << UDRIE0); // turn off interrupt
	UCSR0B &= ~(1 << TXEN0); // turn off tx
	tx_busy = 0; // set to available
}

void usart_stop_rx(void) {
	UCSR0B &= ~(1 << RXCIE0); // turn off interrupt
	UCSR0B &= ~(1 << RXEN0); // turn off rx
	rx_busy = 0; // set to available
}

void usart_send_byte(uint8_t data) {
	UDR0 = data; // load data byte to buffer
}

uint8_t usart_read_byte(void) {
	return UDR0; // read data byte from buffer
}

void usart_send(uint8_t *data, uint8_t i) {
	while(tx_busy); // wait for available
	tx_busy = 1; // set to busy
	usart_tx_buffer_length = i; // set length
	for (uint8_t x = 0; x < usart_tx_buffer_length; x++) { // load data to buffer
		usart_tx_buffer[x] = data[x];
	}
	UCSR0B |= (1 << TXEN0); // turn on tx
	UCSR0B |= (1 << UDRIE0); // turn on interrupt
}

void usart_read(uint8_t max_length) {
	while(rx_busy); // wait for available
	rx_busy = 1; // set to busy
	usart_rx_buffer_length = max_length; // set max length
	usart_rx_complete = 0; // set flag
	UCSR0B |= (1 << RXEN0); // turn on rx
	UCSR0B |= (1 << RXCIE0); // turn on interrupt
}

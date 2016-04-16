#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

static inline void initInterrupt0(); // triggers every 10ms
static inline void initTimer0(); // triggers some time from int0
static inline void startTimer();  // start timing

#define TIM_CLOCK_BITS (1 << CS02) // /256 prescaler

// volatile int
volatile uint8_t dimLevel = 5;

ISR(TIMER0_COMPA_vect) {
	// trigger triac
	PORTB |= (1 << 0);
	_delay_us(5);
	PORTB &= ~(1 << 0);
	// turn off timer
	TCCR0B &= ~TIM_CLOCK_BITS;
}
ISR(INT0_vect) {
	startTimer();
}

int main() {
	// setup
	DDRB = 0xff; // setting up control pin
	// inits
	initInterrupt0();
	initTimer0();
	sei(); // START INTERRUPTS FFS

	// loop
	while(1) {
	}

	return 0;
}

static inline void initTimer0() {
	TCCR0A |= (1 << WGM01); // CTC mode
	TIMSK0 |= (1 << OCIE0A); // enable interrupt
}

static inline void initInterrupt0() {
	EICRA |= (1 << ISC01); // trigger on falling edge
	EIMSK |= (1 << INT0); // enable interrupt
}

static inline void startTimer() {
	TCNT0 = 0; // zero timer
	OCR0A = dimLevel	; // set dimming level
	TCCR0B |= TIM_CLOCK_BITS; // turn on timer
}

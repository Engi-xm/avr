#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <util/delay.h>

static inline void initInterrupt0(); // triggers every 10ms
static inline void initTimer0(); // triggers some time from int0
static inline void startTimer();  // start timing
static inline void initADC1(); // initialize ADC1
static inline uint16_t readADC1(); // return 10bit ADC1 value

#define TIM_CLOCK_BITS (1 << CS02) // /256 clock prescaler

// volatile int
volatile uint8_t dimLevel = 20; // light dimming level

ISR(TIM0_COMPA_vect) {
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
	DDRB |= (1 << 0); // setting up control pin
	uint16_t adcValue; 
	// inits
	initInterrupt0();
	initTimer0();
	initADC1();
	sei(); // START INTERRUPTS FFS

	// loop
	while(1) {
		adcValue = readADC1(); // get pot value

		dimLevel = 8 + (adcValue >> 6); // set dimLevel between 8-23
		if (dimLevel < 11) {
			PORTB |= (1 << 0); // turn triac on
			cli(); // turn off interrupts
		} else if (dimLevel > 20) {
			PORTB &= ~(1 << 0); // turn triac off
			cli(); // turn off interrupts
		} else {
			sei(); // turn on interrupts
		}

		_delay_ms(50); // fuck around
	}

	return 0;
}

static inline void initInterrupt0() {
	MCUCR |= (1 << ISC01); // trigger on falling edg
	GIMSK |= (1 << INT0); // enable interrupt
}

static inline void initTimer0() {
	TCCR0A |= (1 << WGM01); // CTC mode
	TIMSK0 |= (1 << OCIE0A); // enable interrupt
}

static inline void initADC1() {
	ADMUX |= (1 << MUX0); // select channel 1
	ADCSRA |= (1 << ADPS2); // /16 clock prescaler
	ADCSRA |= (1 << ADEN); // enable ADC
}

static inline void startTimer() {
	TCNT0 = 0; // zero timer
	OCR0A = dimLevel	; // set dimming level
	TCCR0B |= TIM_CLOCK_BITS; // turn on timer
}

static inline uint16_t readADC1() {
	ADCSRA |= (1 << ADSC); // start conversion
	loop_until_bit_is_clear(ADCSRA, ADSC); // wait for conversion
	return(ADC); // return 10bit value
}

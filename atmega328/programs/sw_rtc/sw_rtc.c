#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

static inline void initTimer0();
static inline void everySecond();
static inline void everyMinute();
static inline void everyHour();

// times Timer0 overflows per second
#define OVERFLOWS_PS 492

// volatile ints
volatile uint8_t seconds = 0;
volatile uint8_t minutes = 0;
volatile uint8_t hours = 0;
volatile uint16_t tick = 0;

// Timer0 overflow interrupt routine
ISR(TIMER0_OVF_vect) {
	tick++;
}

int main()
{
	// setup
	DDRB |= 0xff;

	initTimer0();
	sei(); // ENABLE FUCKING INTERRUPTS

	// loop
	while(1) {
		// check if second passed
		if (tick == OVERFLOWS_PS) {
			tick = 0;
			everySecond();
			PORTB = seconds; // print out seconds in bin
		}
	}

	return 0;
}

static inline void initTimer0() {
	TCCR0B |= (1 << CS01); // /8 prescaler
	TIMSK0 |= (1 << TOIE0); // enable overflow interrupt
}

static inline void everySecond() {
	// count seconds
	seconds++;
	// restart seconds with correction
	if (seconds > 59) {
		seconds = 0;
		everyMinute();
	}
}

static inline void everyMinute() {
	// count minutes
	minutes++;
	// restart minutes
	if (minutes > 59) {
		minutes = 0;
		everyHour();
	}
}

static inline void everyHour() {
	// count hours
	hours++;
	// restart hours
	if (hours > 24) {
		hours = 0;
	}
}

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <util/delay.h>

static inline void initInterrupt0(); // triggers every 10ms
static inline void initTimer0(); // triggers some time from int0
static inline void initTimer1(); // initialize pwn timer
static inline void initADC1(); // initialize ADC1
static inline void startTimer();  // start timing
static inline uint16_t readADC1(); // return 10bit ADC1 value
static inline void readDHT11(uint8_t* temp, uint8_t* hmdty); // read sensor

// defines
#define TIM0_CLOCK_BITS (1 << CS02) // /256 clock prescaler
#define DHT_DDR DDRA
#define DHT_W PORTA
#define DHT_R PINA
#define DHT_PIN 7

// volatile int
volatile uint8_t dimLevel = 200; // light dimming level
volatile uint16_t tick = 0; // variable for tracking time
uint8_t temp = 0;
uint8_t hmdty = 0;

ISR(TIM0_COMPA_vect) {
	// trigger triac
	PORTB |= (1 << 0);
	_delay_us(3);
	PORTB &= ~(1 << 0);
	// turn off timer
	TCCR0B &= ~TIM0_CLOCK_BITS;
}
ISR(TIM1_OVF_vect) {
	if(tick <= 2000) {
		tick++; // increment tick every ~1ms
	} else {	
		tick = 0; // reset tick
		readDHT11(&temp, &hmdty); // read DHT every ~1s
	}
}
ISR(INT0_vect) {
	startTimer();
}

int main() {
	// setup
	DDRA |= ((1 << 5) | (1 << 2)); // setting up pwm pin
	DDRB |= (1 << 0); // setting up control pin
	uint16_t adcValue;
	// inits
	clock_prescale_set(clock_div_1);
	_delay_ms(1500);
	initInterrupt0();
	initTimer0();
	initTimer1();
	initADC1();
	sei(); // START INTERRUPTS FFS
	// loop
	while(1) {
		adcValue = readADC1(); // get pot value

		OCR1B = adcValue; // set pwm output

		dimLevel = (adcValue >> 2); // set dimLevel between 1-256
		if (dimLevel < 10) {
			PORTB |= (1 << 0); // turn triac on
			cli(); // turn off interrupts
		} else if (dimLevel > 240) {
			PORTB &= ~(1 << 0); // turn triac off
			cli(); // turn off interrupts
		} else {
			sei(); // turn on interrupts
		}

		// if(hmdty > 10) {
		// 	PORTA |= (1 << 2);
		// }

		_delay_ms(100); // fuck around
	}

	return 0;
}

static inline void initInterrupt0() {
	MCUCR |= (1 << ISC01); // trigger on falling edge
	GIMSK |= (1 << INT0); // enable interrupt
}

static inline void initTimer0() {
	TCCR0A |= (1 << WGM01); // CTC mode
	TIMSK0 |= (1 << OCIE0A); // enable interrupt
}

static inline void initTimer1() {
	TCCR1A |= ((1 << WGM11) | (1 << WGM10)); // set 10bit fast pwm mode
	TCCR1B |= (1 << WGM12); // set mode pt. 2
	TCCR1B |= (1 << CS11); // /8 clock prescaler
	TCCR1A |= ((1 << COM1B1) | (1 << COM1B0)); // inverting mode on OC1B
	TIMSK1 |= (1 << TOIE1); // enable overflow interrupt
}

static inline void initADC1() {
	ADMUX |= (1 << MUX0); // select channel 1
	ADCSRA |= ((1 << ADPS2) | (1 << ADPS1)); // /64 clock prescaler
	ADCSRA |= (1 << ADEN); // enable ADC
}

static inline void startTimer() {
	TCNT0 = 0; // zero timer
	OCR0A = dimLevel	; // set dimming level
	TCCR0B |= TIM0_CLOCK_BITS; // turn on timer
}

static inline uint16_t readADC1() {
	ADCSRA |= (1 << ADSC); // start conversion
	loop_until_bit_is_clear(ADCSRA, ADSC); // wait for conversion
	return(ADC); // return 10bit value
}

static inline void readDHT11(uint8_t* temp, uint8_t* hmdty) {
	// initialization
	uint8_t data[5];
	uint8_t arrayIndex = 0;
	uint8_t bitIndex = 7;
	uint8_t startTime;
	uint8_t signalTime;

	// clear data buffer
	for (int i = 0; i < 5; i++) {
		data[i] = 0;
	}

	// send request
	DHT_DDR |= (1 << DHT_PIN);
	DHT_W &= ~(1 << DHT_PIN);
	_delay_ms(18);
	cli();	// turn off interrupts for timing
	DHT_W |= (1 << DHT_PIN);
	_delay_us(40);
	DHT_DDR &= ~(1 << DHT_PIN);
	_delay_us(10);

	// wait for response
	loop_until_bit_is_set(DHT_R, DHT_PIN);
	loop_until_bit_is_clear(DHT_R, DHT_PIN);

	// read 40 bits
	for (int i = 0; i < 40; i++) {
		// wait for signal start
		loop_until_bit_is_set(DHT_R, DHT_PIN);

		// time signal
		startTime = TCNT1;
		loop_until_bit_is_clear(DHT_R, DHT_PIN);
		signalTime = TCNT1 - startTime;

		// push high bit if signal longer than 40us
		if (signalTime > 40) {
			data[arrayIndex] |= (1 << bitIndex);
		}

		// increment or reset indexes
		if (bitIndex == 0) {
			bitIndex = 7;
			arrayIndex++;
		} else {
			bitIndex--;
		}
	}

	// re-enable interrupts
	sei();
	
	// if checksum passed, update data
	if (data[4] == (data[0] + data[2])) {
		*temp = data[2];
		*hmdty = data[0];
	}
}

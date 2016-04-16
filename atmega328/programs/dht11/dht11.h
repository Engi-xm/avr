#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// function prototypes
void readDHT11(uint8_t* temp, uint8_t* hmdty); // read sensor

// defines
#ifndef DHT_TIM // need 1us timer
	#define DHT_TIM '1' // default to timer1
#endif
#ifdef DHT_TIM
	#if (DHT_TIM == '0')
		#define DHT_TIM_CNT TCNT0
	#elif (DHT_TIM == '1')
		#define DHT_TIM_CNT TCNT1
	#endif
	#elif (DHT_TIM == '2')
		#define DHT_TIM_CNT TCNT2
	#endif
#endif
#ifndef DHT_PORT
	#define DHT_PORT 'B' // default to B port
#endif
#ifndef DHT_PIN
	#define DHT_PIN 1 // default to pin 1
#endif
#ifdef DHT_PORT
	#if (DHT_PORT == 'A')
		#define DHT_DDR DDRA
		#define DHT_W PORTA
		#define DHT_R PINA
	#endif
	#if (DHT_PORT == 'B')
		#define DHT_DDR DDRB
		#define DHT_W PORTB
		#define DHT_R PINB
	#endif
	#if (DHT_PORT == 'C')
		#define DHT_DDR DDRC
		#define DHT_W PORTC
		#define DHT_R PINC
	#endif
	#if (DHT_PORT == 'D')
		#define DHT_DDR DDRD
		#define DHT_W PORTD
		#define DHT_R PIND
	#endif
#endif

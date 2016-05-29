#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// function prototypes
void read_dht11(uint8_t* temp, uint8_t* hmdty); // read sensor

// defines
#define DHT_TIM 1 // set timer, needs 1us timer
#define DHT_PIN 7 // st pin
#define DHT_PORT D // set port

#define DHT_TIM_PASTE(x) TCNT ## x
#define DHT_TIM_EVAL(x) DHT_TIM_PASTE(x)
#define DHT_TIM_CNT DHT_TIM_EVAL(DHT_TIM)

#define DHT_DDR_PASTE(x) DDR ## x
#define DHT_DDR_EVAL(x) DHT_DDR_PASTE(x)
#define DHT_DDR DHT_DDR_EVAL(DHT_PORT)

#define DHT_R_PASTE(x) PIN ## x
#define DHT_R_EVAL(x) DHT_R_PASTE(x)
#define DHT_R DHT_R_EVAL(DHT_PORT)

#define DHT_W_PASTE(x) PORT ## x
#define DHT_W_EVAL(x) DHT_W_PASTE(x)
#define DHT_W DHT_W_EVAL(DHT_PORT)

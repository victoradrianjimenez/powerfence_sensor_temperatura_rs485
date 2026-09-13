#include <avr/interrupt.h>
#include "timer.h"

// Configuración del Timer1
#define TIMER_PRESCALER ((1<<CTC1) | (1 << CS12) | (1 << CS11) | (1 << CS10)) // 64
#define TIMER_TOP_VALUE (124) // 8 MHz / 64 / 9600 - 1 = 124

// Variables globales para timer
static volatile uint16_t current_ts = 0; //marca de tiempo actual [ms]

void timer_init() {
    // configurar/habilitar timer1
    TCCR1 = TIMER_PRESCALER;       // CTC mode (clear on OCR1C), normal port operation
    OCR1A = OCR1C = TIMER_TOP_VALUE; // configuro para modo timer
    TIMSK |= (1 << OCIE1A);        // iterrupciones para modo timer
    sei();
}

// Timer1 con OCR1A usado por timer ms.
ISR(TIMER1_COMPA_vect) {
    current_ts++;
}

// Obtener timestamp actual [ms].
uint16_t timer_get_time_ms(){
    cli();
    uint16_t t = current_ts;
    sei();
    return t;
}

// Determina el tiempo transcurrido desde un timestamp [ms].
uint16_t timer_diff_ms(uint16_t last_ts){
    cli();
    uint16_t t = current_ts;
    sei();
    return t - last_ts;
}

// blink a led every 1000ms
void test_timer(uint8_t led_pin) {
    // activo pin como salida
    DDRB |= (1 << led_pin);
    timer_init();
    uint16_t ts = timer_get_time_ms();
    while(1){
        // verificar si llego un frame completo
        if (timer_diff_ms(ts) >= 1000){
            ts += 1000;
            PORTB ^= (1 << led_pin);
        }
    }
}

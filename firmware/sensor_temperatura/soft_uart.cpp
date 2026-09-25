#include <avr/interrupt.h>
#include "soft_uart.h"

// Configuración del Timer0 para implemtación de UART (38400 en adelante es inestable)
#define UART_PRESCALER_2400 ((1 << CS01) | (1 << CS00)) // 64
#define UART_TOP_VALUE_2400 (51) // 8000000 / 64 / 2400 - 1 = 51,0833
#define UART_PRESCALER_4800 (1 << CS01) // 8
#define UART_TOP_VALUE_4800 (207) // 8000000 / 8 / 4800 - 1 = 207,3333
#define UART_PRESCALER_9600 (1 << CS01) // 8
#define UART_TOP_VALUE_9600 (103) // 8000000 / 8 / 9600 - 1 = 103,1667
#define UART_PRESCALER_14400 (1 << CS01) // 8
#define UART_TOP_VALUE_14400 (68) // 8000000 / 8 / 14400 - 1 = 68,4444
#define UART_PRESCALER_19200 (1 << CS01) // 8
#define UART_TOP_VALUE_19200 (51) // 8000000 / 8 / 19200 - 1 = 51,0833
#define UART_PRESCALER_8 (1 << CS01)
#define UART_PRESCALER_64 ((1 << CS01) | (1 << CS00))

#define UART_RX_MASK (1 << UART_RX)
#define UART_TX_MASK (1 << UART_TX)

//#define INVALID_BYTE_VALUE (0xFF)

// Variables globales para UART
static uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE]; // buffer de datos
static uint8_t uart_rx_tail = 0; // posicion de lectura
static volatile uint8_t uart_rx_head = 0; // posicion de escritura
//static uint8_t uart_rx_bit_count; // RX state machine variable
//static volatile uint8_t uart_tx_bit_count; // TX state machine variables
#define uart_rx_bit_count GPIOR0
#define uart_tx_bit_count GPIOR1
//static uint8_t uart0_rx_byte; // RX state machine variable
#define uart_rx_byte GPIOR2 
static volatile uint16_t uart_tx_data; // holds full TX frame (start + data + stop bits)
static volatile uint8_t uart_half_value; // Half bit time used to sample in the middle of the bit

void uart_init(baud_rate_t b) {
    // PINB como salida (1)
    DDRB |= UART_TX_MASK;
    PORTB |= UART_TX_MASK; // idle high (UART idle)
    // PINB como entrada (0)
    DDRB &= ~UART_RX_MASK; 
    PORTB |= UART_RX_MASK; // habilita pull-up interno
#ifdef UART_DE   
    DDRB |= (1 << UART_DE);
    PORTB &= ~(1 << UART_DE); // poner en bajo (RE por defecto)
#endif
    // configurar/habilitar timer0
    TCCR0A = (1 << WGM01); // CTC mode, normal port operation
    soft_uart_set_baud_rate(b);

    // interrupciones de pines de entrada
    GIFR = (1 << PCIF); // clear pin change interrupt flag.
    GIMSK |= (1 << PCIE); // enable external interrupts
    PCMSK |= UART_RX_MASK; // enable pin change on pin RX
    sei();
}

baud_rate_t soft_uart_set_baud_rate(baud_rate_t b){
    uint8_t top;
    if (b == BAUD_RATE_2400){
        TCCR0B = UART_PRESCALER_64;
        top = UART_TOP_VALUE_2400;
    } else {
        switch (b){
        case BAUD_RATE_4800:
            top = UART_TOP_VALUE_4800;
            break;
        case BAUD_RATE_14400:
            top = UART_TOP_VALUE_14400;
            break;
        case BAUD_RATE_19200:
            top = UART_TOP_VALUE_19200;
            break;
        default:
            top = UART_TOP_VALUE_9600;
            b = BAUD_RATE_9600;
        }
    }
    TCCR0B = UART_PRESCALER_8;
    OCR0A = top;
    uart_half_value = top >> 1;
    return b;
}

// Interrupción del pin UART0 RX (ambos flancos) para detección del start bit
ISR(PCINT0_vect) {
    // Ignore rising edge
    if (PINB & UART_RX_MASK) return;

    // Schedule first sample at half bit time
    uint8_t tcnt = TCNT0;
    uint8_t half = uart_half_value;
    OCR0B = (tcnt > half) ? tcnt - half - 1 : tcnt + half;
    
    // Enable RX sampling interrupt
    TIFR = (1 << OCF0B);
    TIMSK |= (1 << OCIE0B);
    // Prepare for next frame
    uart_rx_bit_count = 9;
    // Disable pin change interrupt during reception
    GIMSK &= ~(1 << PCIE);
}

// Interrupción TIMER0 COMPB para RX de UART0 (activa solo durante la recepción)
ISR(TIMER0_COMPB_vect) {
    // Reading start bit + 8 data bits
    if (uart_rx_bit_count--){
        uart_rx_byte = (PINB & UART_RX_MASK) ? (uart_rx_byte >> 1) | 0x80 : (uart_rx_byte >> 1);
        return;
    }
    // Stop bit validation
    if (PINB & UART_RX_MASK){
        // Store byte if buffer not full
        uint8_t pos = uart_rx_head;
        uart_rx_buffer[pos] = uart_rx_byte;
        uart_rx_head = (pos + 1) & (UART_RX_BUFFER_SIZE - 1);
    }
    // Re-enable start detection
    GIFR = (1 << PCIF);
    GIMSK |= (1 << PCIE);
    // Disable sampling interrupt
    TIMSK &= ~(1 << OCIE0B);
}

// Interrupción TIMER0 COMPA para TX de UART0
ISR(TIMER0_COMPA_vect) {
     // Output current bit
    if (uart_tx_data & 1)
        PORTB |= UART_TX_MASK;
    else
        PORTB &= ~UART_TX_MASK;
    // Shift to next bit
    if (uart_tx_bit_count--){
        uart_tx_data >>= 1;
        return;
    }
    // Transmission finished
    TIMSK &= ~(1 << OCIE0A);
}
 
void soft_uart_send(uint8_t* buf, uint8_t sz){
    while (sz--){
        // Wait while UART is transmitting
        while(TIMSK & ((1 << OCIE0A)));
        // Prepare frame: start + data + stop
        uart_tx_data = ((uint16_t)(*buf++) | 0xFF00) << 1; // el shift es para el start bit
        uart_tx_bit_count = 10; //(i == sz - 1) ? 10 : 9; // 9: la ISR apaga el timer al iniciar el stop bit, 10: la ISR apaga el timer al TERMINAR el stop bit.
        // Start transmission (enable timer interruptions)
        TIFR = (1 << OCF0A);
        cli();
        TIMSK |= (1 << OCIE0A);
        sei();
    }
}

void soft_uart_de(){
#ifdef UART_DE 
    // habilitar DE: RS-485 → TX
    PORTB |= (1 << UART_DE);
#endif
}

void soft_uart_re(){
#ifdef UART_DE 
    // deshabilitar DE: RS-485 → RX
    while(TIMSK & ((1 << OCIE0A))); // wait for the stop bit finish
    PORTB &= ~(1 << UART_DE);
#endif
}

uint8_t soft_uart_available() {
    return uart_rx_head != uart_rx_tail; // (uart_rx_head - uart_rx_tail) & (UART_RX_BUFFER_SIZE - 1);
}

uint8_t soft_uart_read() {
    uint8_t tail = uart_rx_tail;
    //if (tail == uart_rx_head) return INVALID_BYTE_VALUE;
    uint8_t x = uart_rx_buffer[tail];
    uart_rx_tail = (tail + 1) & (UART_RX_BUFFER_SIZE - 1);
    return x;
}

// uart echo (un byte a la vez)
void test_uart() {
    uart_init(BAUD_RATE_9600);
    while (1) {
        if (soft_uart_available()){
            //leer byte y reenviarlo
            uint8_t b = soft_uart_read();
            soft_uart_send(&b, 1);
        }
    }
}

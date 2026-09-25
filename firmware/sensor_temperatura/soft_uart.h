#pragma once

#define UART_RX    PB3
#define UART_TX    PB4
#define UART_DE    PB1

// Tamaño del buffer de recepción - POTENCIA DE 2 !!
#define UART_RX_BUFFER_SIZE (16) // minimo 2

typedef enum {
    BAUD_RATE_2400=0,
    BAUD_RATE_4800,
    BAUD_RATE_9600,
    BAUD_RATE_14400,
    BAUD_RATE_19200,
    BAUD_RATE_COUNT
} baud_rate_t;

/**
 * @brief Inicializar pines y timer.
 */
void uart_init(baud_rate_t b);

/**
 * @brief Establecer baud rate para UART0.
 * @param baud Velocidad en baudios
 */
baud_rate_t soft_uart_set_baud_rate(baud_rate_t b);

/**
 * @brief Determina si hay al menos un byte en buffer.
 * @return Valor 1 si hay al menos un byte disponible.
 */
uint8_t soft_uart_available();

/**
 * @brief Leer un byte y luego lo borra del buffer.
 * @return Devuelve el byte disponible o INVALID_BYTE_VALUE en caso de estar vacío el buffer.
 */
uint8_t soft_uart_read();

/**
 * @brief Enviar una secuencia de bytes (trama).
 */
void soft_uart_send(uint8_t *buf, uint8_t sz);

void soft_uart_de();
void soft_uart_re();

/**
 * @brief Funcion de prueba de la uart (simple echo).
 */
void test_uart();

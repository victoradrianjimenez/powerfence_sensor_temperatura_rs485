#pragma once

void timer_init();

/**
 * @brief Calcular la diferencia de tiempo transcurrido, en milisegundos.
 */
uint16_t timer_diff_ms(uint16_t last_ts);

/**
 * @brief Obtener el timestamp actual en milisegundos.
 */
uint16_t timer_get_time_ms();

/**
 * @brief Funcion de prueba que hace parpadear un LED.
 */
void test_timer(uint8_t led_pin);

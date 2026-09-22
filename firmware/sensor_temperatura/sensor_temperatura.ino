/******************************************************************************
 *  Proyecto:        Sensor de temperatura industrial 1.0
 *  Autor:           Victor Adrian Jimenez 
 *  Contacto:        victoradrianjimenez@gmail.com
 *  Fecha:           <2027-09-12>
 *
 *  Descripción:
 *      ATtiny85 @ 8 MHz internal
 *      MODBUS RTU esclavo (9600 8N1)
 ******************************************************************************/

#include <avr/wdt.h>
#include <avr/sleep.h>
#include "timer.h"
#include "soft_uart.h"
#include "modbus_rtu.h"
#include "htu21d.h"

// Timeout de recepción, en milisegundos
#define HTU21D_MEASURING_TIME (50)
#define HTU21D_MEASURING_TIMEOUT (60)
// Tiempo de inactividad en uart que debe ocurrir antes de enviar datos (evitar solapamiento)
#define UART_IDLE_TIME_MS (10) // Recomendado >= 10ms.
// Cada cuanto tiempo se refrescan las mediciones
#define MEASURING_PERIOD_MS (1000)
// Cuantos intentos se hacen de lectura de sensores
#define REQUEST_RETRIES (3) // reintentos de lectura

enum {
    STATE_IDLE = 0,
    STATE_REQUEST,
    STATE_WAIT,
    STATE_RECEIVE,
    STATE_STORE,
    STATE_FAIL,
};

/**
 * @brief Main function 
 */
int main() {
    //test_timer(PB1);
    //test_uart();
    //test_modbus_rtu();
    //htu21d_test();

    int16_t temp, hum;
    uint8_t repetition = 0, sensor_position = 0, state = STATE_IDLE;
    uint16_t current_ts = 0, request_ts = 0, uart_busy_ts = 0;
    uint16_t last_measurement_ts = -MEASURING_PERIOD_MS; // forzar lectura de sensores

    MCUSR = 0; // limpiar flags de reset (importante: leerlos ANTES si querés diagnosticar la causa)
    wdt_disable();  // apagar cualquier WDT que haya quedado activo de un reset anterior
    
    // apagar perifericos que no uso para ahorro de energía
    ADCSRA &= ~(1 << ADEN); // Disable ADC
    ADMUX = 0;
    ACSR |= (1 << ACD); // Disable Analog Comparator 
    PRR |= (1 << PRADC); // apaga ADC 
    // inicializar timer
    timer_init();
    // inicializar uart
    uart_init(BAUD_RATE_9600);
    // modbus: enviar mensajes por USI UART
    modbus_init();
    // sensor de temperatura y humedad
    htu21d_init();
    //set_sleep_mode(SLEEP_MODE_IDLE);
    //sleep_enable();
    // loop principal
    wdt_enable(WDTO_4S); // habilitar recién al final, justo antes del while(1)
    while (1) {
        wdt_reset();

        // obtengo el tiempo en la iteracion
        current_ts = timer_get_time_ms();

        // comprobar si tengo datos entrantes
        modbus_check_requests();

        // tomar el tiempo para saber cuanto tiempo pasó desde que estuvo ocupada la UART0
        if (soft_uart_busy()) uart_busy_ts = current_ts;
        
        switch (state){
        case STATE_IDLE:
            // esperando que se cumpla el tiempo para iniciar lectura de sensores
            if (timer_diff_ms(last_measurement_ts) >= MEASURING_PERIOD_MS){
                last_measurement_ts += MEASURING_PERIOD_MS; 
                // preparo estado para la lectura del primer sensor
                sensor_position = repetition = 0;
                state = STATE_REQUEST;
                break;
            }
            break;

        case STATE_REQUEST:
            // si falla despues de varios reintentos, indicar estado de falla
            if (repetition >= REQUEST_RETRIES) {
                state = STATE_FAIL;
                break;
            }
            // si UART ha pasado tiempo inactivo (para evitar overlapping)
            if (timer_diff_ms(uart_busy_ts) >= UART_IDLE_TIME_MS){
                repetition++;
                // enviar solicitud
                if (sensor_position == 0){
                    if (!htu21d_request(HTU21D_CMD_TEMP_NOHOLD)){
                        state = STATE_FAIL;
                        break;
                    }
                } else {
                    if (!htu21d_request(HTU21D_CMD_HUM_NOHOLD)){
                        state = STATE_FAIL;
                        break;
                    }
                }
                // entrar en estado de espera de respuesta
                request_ts = current_ts;
                state = STATE_WAIT;
                break;
            }
            break;

        case STATE_WAIT: // esperando respuesta del sensor actual
            if (timer_diff_ms(request_ts) >= HTU21D_MEASURING_TIME){ 
                state = STATE_RECEIVE;
                break;
            }
            break;
            
        case STATE_RECEIVE: // esperando mensaje completo
            if (sensor_position == 0){
                if(htu21d_read_temperature(&temp)){
                    // se ha completado la lectura del sensor e iniciar proximo
                    sensor_position = 1;
                    repetition = 0;
                    state = STATE_REQUEST;                    
                    break;
                }
            } else {
                if(htu21d_read_humidity(&hum)){
                    // se ha completado la lectura de los sensores
                    state = STATE_STORE;
                    break;
                }
            }
            // check timeout
            if (timer_diff_ms(request_ts) >= HTU21D_MEASURING_TIMEOUT){
                state = STATE_REQUEST;
                break;
            }
            break;

        case STATE_STORE: // todos los sensores fueron leidos correctamente
            modbus_set_register(VAR_TEMP, &temp);
            modbus_set_register(VAR_HUM, &hum);
            state = STATE_IDLE;
            break;

        case STATE_FAIL: // uno de los sensores falla despues de N intentos
            htu21d_request(HTU21D_CMD_SOFT_RESET);
            state = STATE_IDLE;
            break;

        default:
            state = STATE_IDLE;
            break;
        }
        //sleep_cpu(); // detiene el reloj del núcleo hasta la próxima interrupción
    }
    return 0;
}

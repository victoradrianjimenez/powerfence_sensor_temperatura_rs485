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
// Cada cuanto tiempo se refrescan las mediciones
#define MEASURING_PERIOD_MS (1000)

enum {
    STATE_IDLE = 0,
    STATE_REQUEST,
    STATE_WAIT,
    STATE_FAIL,
};

/**
 * @brief Main function 
 */
int main(void) {
    //test_timer(PB1);
    //test_uart();
    //test_modbus_rtu();
    //htu21d_test();
    
    int16_t data;
    uint8_t variable, state = STATE_IDLE;
    uint16_t request_ts, aux_ts, last_measurement_ts = -MEASURING_PERIOD_MS; // forzar lectura de sensores

    MCUSR = 0; // limpiar flags de reset (importante: leerlos ANTES si querés diagnosticar la causa)
    wdt_disable();  // apagar cualquier WDT que haya quedado activo de un reset anterior
    
    // apagar perifericos que no uso para ahorro de energía
    ADCSRA &= ~(1 << ADEN); // Disable ADC
    ACSR |= (1 << ACD); // Disable Analog Comparator 
    PRR |= (1 << PRADC) | (1 << PRUSI); // apaga ADC y // USI
    // inicializar timer
    timer_init();
    // inicializar uart
    uart_init(BAUD_RATE_9600);
    // modbus: enviar mensajes por USI UART
    modbus_init();
    // sensor de temperatura y humedad
    htu21d_init();
    // en IDLE se detiene solo el reloj de la CPU:
    //set_sleep_mode(SLEEP_MODE_IDLE);
    //sleep_enable();
    // loop principal
    wdt_enable(WDTO_4S); // habilitar recién al final, justo antes del while(1)
    while (1) {
        wdt_reset();

        // comprobar si tengo datos entrantes
        modbus_check_requests();

        switch (state){
        case STATE_IDLE:
            // esperando que se cumpla el tiempo para iniciar lectura de sensores
            if (timer_get_time_ms(last_measurement_ts) < MEASURING_PERIOD_MS) break;
            last_measurement_ts += MEASURING_PERIOD_MS; 
            // preparo estado para la lectura del primer sensor
            variable = HTU21D_CMD_TEMP_NOHOLD;
            state = STATE_REQUEST;
            break;

        case STATE_REQUEST:
            // enviar solicitud
            if (!htu21d_request(variable)){
                state = STATE_FAIL;
                break;
            }
            // entrar en estado de espera de respuesta
            request_ts = timer_get_time_ms(0);
            state = STATE_WAIT;
            break;

        case STATE_WAIT: // esperando respuesta del sensor actual
            aux_ts = timer_get_time_ms(request_ts);
            if (aux_ts >= HTU21D_MEASURING_TIMEOUT){
                state = STATE_FAIL;
                break;
            }
            if (aux_ts >= HTU21D_MEASURING_TIME) break;
            if (variable == HTU21D_CMD_TEMP_NOHOLD){
                if(htu21d_read_temperature(&data)){
                    // se ha completado la lectura del sensor e iniciar proximo
                    modbus_set_register(VAR_TEMP, data);
                    variable = HTU21D_CMD_HUM_NOHOLD;
                    state = STATE_REQUEST;                    
                    break;
                }
            } else {
                if(htu21d_read_humidity(&data)){
                    // se ha completado la lectura de los sensores
                    modbus_set_register(VAR_HUM, data);
                    state = STATE_IDLE;
                    break;
                }
            }
            break;
            
        default: // uno de los sensores falla despues de N intentos
            htu21d_request(HTU21D_CMD_SOFT_RESET);
            state = STATE_IDLE;
        }
        //sleep_cpu();
    }
    return 0;
}

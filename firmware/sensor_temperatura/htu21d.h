#pragma once
#include <stdint.h>

#define SDA_PIN   PB0
#define SCL_PIN   PB2

#define HTU21D_ADDR            0x40
#define HTU21D_CMD_TEMP_NOHOLD 0xF3
#define HTU21D_CMD_HUM_NOHOLD  0xF5
#define HTU21D_CMD_SOFT_RESET  0xFE

/**
 * @brief Iniciar driver
 */
void htu21d_init(void);

uint8_t htu21d_request(uint8_t cmd);

/**
 * @brief Temperatura en °C. Devuelve 1 si la lectura fue exitosa.
 */
uint8_t htu21d_read_temperature(int16_t *temp_c100);
 
/**
 * @brief Humedad relativa en %. Devuelve 1 si la lectura fue exitosa.
 */
uint8_t htu21d_read_humidity(int16_t *humidity_rh100);

void htu21d_test(void);
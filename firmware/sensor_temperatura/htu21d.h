#pragma once

#define SDA_PIN   PB0
#define SCL_PIN   PB2

#define HTU21D_ADDR          0x40
#define HTU21D_CMD_TEMP_NOHOLD 0xF3
#define HTU21D_CMD_HUM_NOHOLD  0xF5
#define HTU21D_CMD_SOFT_RESET  0xFE

/**
 * @brief Iniciar driver
 */
void htu21d_init(void);

/**
 * @brief Intentar lectura (mientras mide, el HTU21D responde NACK a su dirección)
 */
uint8_t htu21d_read(uint16_t *raw_out);
 
/**
 * @brief Temperatura en °C. Devuelve 1 si la lectura fue exitosa.
 */
uint8_t htu21d_request_temperature();
uint8_t htu21d_read_temperature(float *temp_c);
 
/**
 * @brief Humedad relativa en %. Devuelve 1 si la lectura fue exitosa.
 */
uint8_t htu21d_request_humidity();
uint8_t htu21d_read_humidity(float *humidity_rh);

/**
 * @brief Reinicia el sensor (recomendado al inicio, tarda ~15ms en estar listo)
 */
uint8_t htu21d_reset(void);

void htu21d_test();
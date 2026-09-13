#ifndef F_CPU
#define F_CPU 8000000UL
#endif

//#include <avr/pgmspace.h>
#include <avr/io.h>
#include <util/delay.h>
#include "htu21d.h"

#define I2C_DELAY() _delay_us(5)

// Tabla de nibbles para CRC-8 del HTU21D (poly 0x31, MSB-first, sin reflexión).
static const uint8_t crc8_nibble_table[16] = {
    0x00, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x97,
    0xB9, 0x88, 0xDB, 0xEA, 0x7D, 0x4C, 0x1F, 0x2E
};
 
// Calcula el CRC-8 del HTU21D sobre los 2 bytes de dato (MSB, LSB), procesando un nibble (4 bits) a la vez en vez de bit a bit.
static uint8_t htu21d_crc8(uint8_t msb, uint8_t lsb) {
    uint8_t crc = msb;
    crc = (uint8_t)((crc << 4) ^ crc8_nibble_table[(crc >> 4) & 0x0F]); // nibble alto
    crc = (uint8_t)((crc << 4) ^ crc8_nibble_table[(crc >> 4) & 0x0F]); // nibble bajo
    crc ^= lsb;
    crc = (uint8_t)((crc << 4) ^ crc8_nibble_table[(crc >> 4) & 0x0F]); // nibble alto
    crc = (uint8_t)((crc << 4) ^ crc8_nibble_table[(crc >> 4) & 0x0F]); // nibble bajo
    return crc;
}

static inline uint8_t sda_read(void) { return (PINB & (1 << SDA_PIN)) ? 1 : 0; }
static inline void sda_high(void)  { DDRB  &= ~(1 << SDA_PIN); } // suelta SDA (pull-up la lleva a alto)
static inline void sda_low(void)   { PORTB &= ~(1 << SDA_PIN); DDRB  |= (1 << SDA_PIN); } // Apaga pull-up primero, Luego pasa a salida
static inline void scl_high(void)  { DDRB  &= ~(1 << SCL_PIN); uint8_t timeout = 255; while (!(PINB & (1 << SCL_PIN)) && --timeout); } // clock stretching
static inline void scl_low(void)   { PORTB &= ~(1 << SCL_PIN); DDRB  |= (1 << SCL_PIN); }

static void i2c_start() {
    sda_high();
    scl_high();
    I2C_DELAY();
    sda_low();
    I2C_DELAY();
    scl_low();
    I2C_DELAY(); // necesario?
}

static void i2c_stop() {
    sda_low();
    I2C_DELAY();
    scl_high();
    I2C_DELAY();
    sda_high();
    I2C_DELAY();
}

static uint8_t i2c_write(uint8_t data) {
    for (uint8_t i = 0; i < 8; i++) {
        if (data & 0x80) sda_high(); else sda_low();
        I2C_DELAY();
        scl_high();
        I2C_DELAY();
        scl_low();
        data <<= 1;
    }
    sda_high();
    I2C_DELAY();
    scl_high();
    I2C_DELAY();
    uint8_t ack = !sda_read();
    scl_low();
    return ack;
}

static uint8_t i2c_read_byte(uint8_t send_ack){
    uint8_t data = 0;
    sda_high();
    for (uint8_t i = 0; i < 8; i++){
        data <<= 1;
        I2C_DELAY();
        scl_high();
        I2C_DELAY();
        if (sda_read())
            data |= 1;
        scl_low();
    }
    // Enviar ACK (0) o NACK (1)
    if (send_ack) sda_low(); else sda_high();
    I2C_DELAY();
    scl_high();
    I2C_DELAY();
    scl_low();
    sda_high();
    return data;
}

// Envía el comando de medición
static uint8_t htu21d_request(uint8_t cmd){
    uint8_t ok = 0;
    i2c_start();
    ok = i2c_write((HTU21D_ADDR << 1) | 0);
    if (!ok) goto exit;
    ok = i2c_write(cmd);
exit:
    i2c_stop();
    return ok;
}

// Intentar lectura (mientras mide, el HTU21D responde NACK a su dirección)
static uint8_t htu21d_read(uint16_t *raw_out){
    uint8_t ok = 0;
    i2c_start();
    if (!i2c_write((HTU21D_ADDR << 1) | 1)){ // dirección + lectura
        i2c_stop();
        return 0;
    }
    // Leer 2 bytes de dato (MSB primero) + 1 byte de CRC (lo descartamos aquí)
    uint8_t msb = i2c_read_byte(1); // ACK para pedir el siguiente byte
    uint8_t lsb = i2c_read_byte(1); // ACK para pedir el CRC
    uint8_t crc = i2c_read_byte(0);               // leer CRC y responder NACK (fin de lectura)
    i2c_stop();
    if (htu21d_crc8(msb, lsb) != crc) return 0; // dato corrupto: descartar esta lectura
    uint16_t raw = ((uint16_t)msb << 8) | lsb;
    raw &= 0xFFFC; // los 2 bits menos significativos son bits de estado, no de dato
    *raw_out = raw;
    return 1;
}

// Iniciar driver
void htu21d_init(void){
    // Ambas líneas en alto (liberadas) al inicio
    PORTB |= (1 << SDA_PIN) | (1 << SCL_PIN);
    DDRB  |= (1 << SDA_PIN) | (1 << SCL_PIN);
    USICR = 0; // USI en modo bit-banging manual (no usamos el contador USI aquí)
    sda_high();
    scl_high();
    I2C_DELAY();
}

uint8_t htu21d_request_temperature(){
    return htu21d_request(HTU21D_CMD_TEMP_NOHOLD);
}
 
// Temperatura en °C. Devuelve 1 si la lectura fue exitosa.
uint8_t htu21d_read_temperature(float *temp_c){
    uint16_t raw;
    if (!htu21d_read(&raw)) return 0;
    // Fórmula del datasheet: T = -46.85 + 175.72 * (ST / 2^16)
    *temp_c = -46.85f + 175.72f * ((float)raw / 65536.0f);
    return 1;
}

uint8_t htu21d_request_humidity(){
    return htu21d_request(HTU21D_CMD_HUM_NOHOLD);
}

// Humedad relativa en %. Devuelve 1 si la lectura fue exitosa.
uint8_t htu21d_read_humidity(float *humidity_rh){
    uint16_t raw;
    if (!htu21d_read(&raw)) return 0;
    // Fórmula del datasheet: RH = -6 + 125 * (SRH / 2^16)
    *humidity_rh = -6.0f + 125.0f * ((float)raw / 65536.0f);
    return 1;
}

// Reinicia el sensor (recomendado al inicio, tarda ~15ms en estar listo)
uint8_t htu21d_reset(void){
    uint8_t ok = htu21d_request(HTU21D_CMD_SOFT_RESET);
    if (ok) _delay_ms(15);
    return ok;
}

void htu21d_test(){
    float temp, hum;
    htu21d_init();

    htu21d_reset();

    htu21d_request_temperature();
    _delay_ms(50);
    htu21d_read_temperature(&temp);

    htu21d_request_humidity();
    _delay_ms(50);
    htu21d_read_humidity(&temp);

}

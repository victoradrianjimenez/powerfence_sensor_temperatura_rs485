#ifndef F_CPU
#define F_CPU 8000000UL
#endif

//#include <avr/pgmspace.h>
#include <avr/io.h>
#include <util/delay.h>
#include "htu21d.h"
#include "soft_uart.h"
#include <stdlib.h> // para dtostrf
#include <string.h> // para strlen

#define I2C_DELAY() _delay_us(5)

static uint8_t htu21d_crc8(uint8_t msb, uint8_t lsb) {
    uint8_t crc = 0;
    uint8_t data[2] = { msb, lsb };
    for (uint8_t i = 0; i < 2; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
        }
    }
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
uint8_t htu21d_request(uint8_t cmd){
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
    uint8_t crc = i2c_read_byte(0); // leer CRC y responder NACK (fin de lectura)

    i2c_stop();
    if (htu21d_crc8(msb, lsb) != crc) return 0; // dato corrupto: descartar esta lectura
    uint16_t raw = ((uint16_t)msb << 8) | lsb;
    *raw_out = raw & 0xFFFC; // los 2 bits menos significativos son bits de estado, no de dato
    return 1;
}

// Iniciar driver
void htu21d_init(void){
    // Ambas líneas en alto (liberadas) al inicio
    PORTB |= (1 << SDA_PIN) | (1 << SCL_PIN);
    DDRB  |= (1 << SDA_PIN) | (1 << SCL_PIN);
    sda_high();
    scl_high();
    I2C_DELAY();
}
 
// Temperatura en °C. Devuelve 1 si la lectura fue exitosa. 
uint8_t htu21d_read_temperature(int16_t *temp_c100){
    // T = -46.85 + 175.72 * (raw / 65536), en centesimas de grado C
    uint16_t raw;
    if (!htu21d_read(&raw)) return 0;
    int32_t t = ((int32_t)raw * 17572L) >> 16;  // 175.72 * 100 / 65536
    *temp_c100 = (int16_t)(t - 4685);           // -46.85 * 100
    return 1;
}

// Humedad relativa en %. Devuelve 1 si la lectura fue exitosa.
uint8_t htu21d_read_humidity(int16_t *humidity_rh100){
    // RH = -6 + 125 * (raw / 65536), devuelto como centesimas de %HR (int16, valor x100)
    uint16_t raw;
    if (!htu21d_read(&raw)) return 0;
    int32_t h = ((int32_t)raw * 12500L) >> 16;   // 125 * 100 / 65536
    *humidity_rh100 = (int16_t)(h - 600);        // -6 * 100 = -600
    return 1;
}

void htu21d_test(){
    char data[10];
    uint8_t res;
    uint16_t temp, hum;
    uart_init(BAUD_RATE_9600);
    htu21d_init();
    res = htu21d_request(HTU21D_CMD_TEMP_NOHOLD);
    if (res){ 
        _delay_ms(50);
        res = htu21d_read_temperature(&temp);
        if (res){
            data[0] = 'T';
            dtostrf(temp, 1, 2, &data[1]); // ancho mínimo 1, 2 decimales
            soft_uart_send(data, strlen(data));
        }
    }
    res = htu21d_request(HTU21D_CMD_HUM_NOHOLD);
    if (res){
        _delay_ms(50);
        res = htu21d_read_humidity(&hum);
        if (res){
            data[0] = 'H';
            dtostrf(hum, 1, 2, &data[1]); // ancho mínimo 1, 2 decimales
            soft_uart_send(data, strlen(data));        
        }
    }
    res = htu21d_request(HTU21D_CMD_SOFT_RESET);
    if (res){
        _delay_ms(15);
        data[0] = 'R';
        data[1] = '\0';
        soft_uart_send(data, strlen(data));
    }
}

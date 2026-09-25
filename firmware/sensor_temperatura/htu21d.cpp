#ifndef F_CPU
#define F_CPU 8000000UL
#endif

//#include <avr/pgmspace.h>
#include <avr/io.h>
#include <util/delay.h>
#include "htu21d.h"
#include "soft_uart.h"
#include "timer.h"

#define SDA_MASK (1 << SDA_PIN)
#define SCL_MASK (1 << SCL_PIN)
#define I2C_DELAY() _delay_us(5)

// CRC-8 del HTU21D (poly 0x31, init 0), un byte por llamada
static uint8_t crc8_step(uint8_t crc, uint8_t d) {
    crc ^= d;
    for (uint8_t b = 0; b < 8; b++) {
        crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
    return crc;
}

static inline uint8_t sda_read(void) { return (PINB & SDA_MASK) ? 1 : 0; }
static inline void sda_low(void)   { PORTB &= ~SDA_MASK; DDRB  |= SDA_MASK; } // Apaga pull-up primero, Luego pasa a salida
static inline void sda_high(void)  { DDRB  &= ~SDA_MASK; } // suelta SDA (pull-up la lleva a alto)
static inline void scl_low(void)   { PORTB &= ~SCL_MASK; DDRB  |= SCL_MASK; }
static __attribute__((noinline)) void scl_high(void)  { DDRB  &= ~SCL_MASK; uint8_t timeout = 255; while (!(PINB & SCL_MASK) && --timeout); } // clock stretching

static void i2c_start(void) {
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

// Un ciclo de reloj: pone 'bit' en SDA, pulsa SCL y devuelve el nivel leído en SDA.
// Sirve para escribir (se ignora el valor leído) y para leer (bit = 1 suelta SDA).
static uint8_t i2c_bit(uint8_t bit) {
    if (bit) sda_high(); else sda_low();
    I2C_DELAY();
    scl_high();
    I2C_DELAY();
    uint8_t r = sda_read();
    scl_low();
    return r;
}

static uint8_t i2c_write(uint8_t data) {
    for (uint8_t i = 0; i < 8; i++) {
        i2c_bit(data & 0x80);
        data <<= 1;
    }
    return !i2c_bit(1); // ACK del esclavo = SDA en bajo
}

static uint8_t i2c_read_byte(uint8_t send_ack){
    uint8_t data = 0;
    sda_high();
    for (uint8_t i = 0; i < 8; i++){
        data = (data << 1) | i2c_bit(1);
    }
    i2c_bit(!send_ack); // ACK (0) o NACK (1)
    sda_high();
    return data;
}

// Envía el comando de medición
uint8_t htu21d_request(uint8_t cmd){
    i2c_start();
    uint8_t ok = i2c_write(HTU21D_ADDR << 1) && i2c_write(cmd);
    i2c_stop();
    return ok;
}

// Intentar lectura (mientras mide, el HTU21D responde NACK a su dirección)
static uint8_t htu21d_read(uint16_t *raw_out){
    uint8_t buf[3];
    uint8_t crc = 0;
    i2c_start();
    if (!i2c_write((HTU21D_ADDR << 1) | 1)){ // dirección + lectura
        i2c_stop();
        return 0;
    }
    // MSB, LSB y CRC. ACK en los dos primeros, NACK tras el CRC (fin de lectura)
    for (uint8_t i = 0; i < 3; i++) {
        buf[i] = i2c_read_byte(i < 2);
        crc = crc8_step(crc, buf[i]);
    }
    i2c_stop();
    if (crc) return 0; // el CRC calculado sobre datos + CRC recibido debe dar 0
    *raw_out = (((uint16_t)buf[0] << 8) | buf[1]) & 0xFFFC; // los 2 bits menos significativos son bits de estado, no de dato
    return 1;
}

// Iniciar driver: líneas liberadas con pull-up interno.
// Se necesitan pull-ups externos: el interno se apaga la primera vez que se baja la línea.
void htu21d_init(void) {
    DDRB  &= ~(SDA_MASK | SCL_MASK);
    PORTB |= SDA_MASK | SCL_MASK;
    I2C_DELAY();
}
 
// out = raw * mult / 65536 - offset   (una sola multiplicación de 32 bits para T y RH)
static uint8_t htu21d_read_scaled(uint16_t mult, int16_t offset, int16_t *out) {
    uint16_t raw;
    if (!htu21d_read(&raw)) return 0;
    *out = (int16_t)(((uint32_t)raw * mult) >> 16) - offset;
    return 1;
}

// Temperatura en centésimas de °C. Devuelve 1 si la lectura fue exitosa.
// T = -46.85 + 175.72 * (raw / 65536)
uint8_t htu21d_read_temperature(int16_t *temp_c100) {
    return htu21d_read_scaled(17572, 4685, temp_c100);
}

// Humedad relativa en centésimas de %HR. Devuelve 1 si la lectura fue exitosa.
// RH = -6 + 125 * (raw / 65536)
uint8_t htu21d_read_humidity(int16_t *humidity_rh100) {
    return htu21d_read_scaled(12500, 600, humidity_rh100);
}

void htu21d_test(void){
    uint8_t res, data;
    uint16_t temp, hum;
    uint16_t ts = 0;
    timer_init();
    uart_init(BAUD_RATE_9600);
    htu21d_init();
    while(1){
        if (timer_get_time_ms(ts) >= 1000){
            ts += 1000;
            res = htu21d_request(HTU21D_CMD_TEMP_NOHOLD);
            if (res){ 
                _delay_ms(50);
                res = htu21d_read_temperature(&temp);
                if (res){
                    data = (uint8_t) ((temp / 100) & 0x00FF);
                    soft_uart_send(&data, 1);
                }
            }
            res = htu21d_request(HTU21D_CMD_HUM_NOHOLD);
            if (res){
                _delay_ms(50);
                res = htu21d_read_humidity(&hum);
                if (res){
                    data = (uint8_t) ((hum / 100) & 0x00FF);
                    soft_uart_send(&data, 1); 
                }
            }
            res = htu21d_request(HTU21D_CMD_SOFT_RESET);
            if (res){
                _delay_ms(15);
                data = (uint8_t)'R';
                soft_uart_send((uint8_t*)&data, 1);
            }
        }
    }
}

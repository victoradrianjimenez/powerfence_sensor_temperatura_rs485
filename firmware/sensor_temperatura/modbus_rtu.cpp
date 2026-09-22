// docs: https://www.beijerelectronics.com/docs/DIO/GL-997X/en/modbus-interface.html

#ifndef MODBUS_LITE_VERSION
#include <avr/eeprom.h>
#endif
#include <util/delay.h>
#include "soft_uart.h"
#include "modbus_rtu.h"

// Tamaño del buffer local
#define BUFFER_SIZE (8) // suficiente para almacenar el mensaje modbus más largo

typedef union {
    uint16_t u16;
    uint8_t u8[2];
} my_uint16;

#ifndef MODBUS_LITE_VERSION
static const uint8_t config_register_map[CONF_POS_COUNT] = {
    REG_ADDR_FIRMWARE_VERSION,
    REG_ADDR_SLAVE_ADDRESS,
    REG_ADDR_BAUD_RATE,
    REG_ADDR_PARITY,
    REG_ADDR_STOP_BITS
};
static uint16_t config_registers[CONF_POS_COUNT] EEMEM;   // en memoria no volatil
#endif

static uint8_t request_pos = 0;
static uint8_t request_buf[BUFFER_SIZE];
static uint16_t holding_registers[VAR_COUNT];

static my_uint16 crc;
static uint8_t slave_address = MODBUS_DEFAULT_SLAVE_ADDRESS;

#define crc16_init(crc) ((crc)->u16 = 0xFFFF)

// CRC-16 Modbus (poly 0xA001, reflejado) bit a bit
static void crc16_step(my_uint16* crc, uint8_t* d, uint8_t n) {
    for (uint8_t i = 0; i < n; i++) {
        crc->u16 ^= d[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc->u16 = (crc->u16 & 1) ? (crc->u16 >> 1) ^ 0xA001 : crc->u16 >> 1;
        }
    }
}

void modbus_init(void){
    // pongo datos en cero
    for (uint8_t i = 0; i < VAR_COUNT; i++){
        holding_registers[i] = 0;
    }
#ifndef MODBUS_LITE_VERSION
    // Si EEPROM nunca fue inicializada, poner valor por defecto
    slave_address = eeprom_read_word(&config_registers[CONF_POS_SLAVE_ADDRESS]);
    if (slave_address > 247 || slave_address == 0x00) {
        slave_address = MODBUS_DEFAULT_SLAVE_ADDRESS; // valor por defecto
        eeprom_update_word(&config_registers[CONF_POS_SLAVE_ADDRESS], MODBUS_DEFAULT_SLAVE_ADDRESS);
    }
    baud_rate_t baudrate = (baud_rate_t)eeprom_read_word(&config_registers[CONF_POS_BAUD_RATE]);
    if (soft_uart_check_baud_rate(baudrate)){
        soft_uart_set_baud_rate(baudrate);
    } else {
        eeprom_update_word(&config_registers[CONF_POS_BAUD_RATE], MODBUS_DEFAULT_BAUD_RATE);
    }
    if (eeprom_read_word(&config_registers[CONF_POS_FIRMWARE_VERSION]) != FIRMWARE_VERSION){
        eeprom_update_word(&config_registers[CONF_POS_FIRMWARE_VERSION], FIRMWARE_VERSION);
    }
    if (eeprom_read_word(&config_registers[CONF_POS_PARITY]) != MODBUS_DEFAULT_PARITY){
        eeprom_update_word(&config_registers[CONF_POS_PARITY], MODBUS_DEFAULT_PARITY);
    }
    if (eeprom_read_word(&config_registers[CONF_POS_STOP_BITS]) != MODBUS_DEFAULT_STOP_BITS){
        eeprom_update_word(&config_registers[CONF_POS_STOP_BITS], MODBUS_DEFAULT_STOP_BITS);
    }
#endif
}

static void modbus_send_exception(uint8_t exc_code) {
    uint8_t out[5] = {
        slave_address,
        (uint8_t)(request_buf[1] | 0x80),
        exc_code,
    };
    crc16_init(&crc);
    crc16_step(&crc, out, 3);
    out[3] = crc.u8[0];
    out[4] = crc.u8[1];
    soft_uart_de();
    soft_uart_send(out, sizeof(out));
    soft_uart_re();
}

static inline void modbus_send_echo(void){
    soft_uart_de();
    soft_uart_send(request_buf, sizeof(request_buf));
    soft_uart_re();
}

#ifndef MODBUS_LITE_VERSION
static inline uint8_t modbus_read_config_registers(uint16_t addr, uint8_t nregs){
    // preparo respuesta con registros
    uint8_t j;
    for (uint8_t i = 0; i < nregs; i++){
        for (j = 0; j < CONF_POS_COUNT; j++){
            if (config_register_map[j] == addr + i) break;
        }
        if (j == CONF_POS_COUNT) return 0;
    }
    // enviar respuesta con valor del registro
    crc16_init(&crc);
    my_uint16 val;
    uint8_t out[3] = {
        slave_address, // slave address
        MODBUS_FUNCTION_READ_HOLDING_REGISTERS, // command
        (uint8_t)(nregs << 1)     // number of bytes
    };
    crc16_step(&crc, out, 3);
    soft_uart_de();
    soft_uart_send(out, 3);
    j = 0;
    while(nregs-- > 0){
        // buscar posicion del registro en base a su address
        while (j < CONF_POS_COUNT && config_register_map[j] != addr) j++;
        val.u16 = eeprom_read_word(&config_registers[j]);
        out[0] = val.u8[1]; //MSB
        out[1] = val.u8[0]; //LSB
        soft_uart_send(out, 2);
        crc16_step(&crc, out, 2);
        addr++;
    }
    soft_uart_send(crc.u8, 2);
    soft_uart_re();
    return 1;
}
#endif

static inline void modbus_read_holding_registers(void){
    my_uint16 addr = {.u8 = {request_buf[3], request_buf[2]}};
    uint8_t nregs = request_buf[5]; // uint16_t nregs = (request_buf[4] << 8) | request_buf[5];
    if (request_buf[4] != 0 || nregs == 0 || nregs > 125) {
        modbus_send_exception(MDOBUS_EXC_CODE_ILEGAL_DATA); //ilegal address
        return; // hasta 256 elementos
    }
    // verificar direccion
    if (!(addr.u16 >= 0 && addr.u16 + nregs <= VAR_COUNT)){
#ifndef MODBUS_LITE_VERSION
        if (!modbus_read_config_registers(addr.u16, nregs))
#endif
            modbus_send_exception(MDOBUS_EXC_CODE_ILEGAL_ADDRESS); //ilegal address
        return;
    }
    // preparo respuesta con registros
    crc16_init(&crc);
    my_uint16 val;
    uint8_t out[3] = {
        slave_address, // slave address
        request_buf[1], // command
        (uint8_t)(nregs << 1),    // number of bytes
    };
    crc16_step(&crc, out, 3);
    soft_uart_de();
    soft_uart_send(out, 3);
    while(nregs-- > 0){
        val.u16 = holding_registers[addr.u16++];
        out[0] = val.u8[1];
        out[1] = val.u8[0];
        soft_uart_send(out, 2);
        crc16_step(&crc, out, 2);
    }
    soft_uart_send(crc.u8, 2);
    soft_uart_re();
}

#ifndef MODBUS_LITE_VERSION
static inline void modbus_write_holding_register(void){
    uint8_t j;
    my_uint16 addr = {.u8 = {request_buf[3], request_buf[2]}};
    // buscar posicion del registro en base a su address
    for (j = 0; j < CONF_POS_COUNT; j++){
        if (config_register_map[j] == addr.u16) break;
    }
    my_uint16 value = {.u8 = {request_buf[5], request_buf[4]}};
    switch (j){
    case CONF_POS_SLAVE_ADDRESS:
        // check value
        if (value.u16 > 247 || value.u16 == 0){
            modbus_send_exception(MDOBUS_EXC_CODE_ILEGAL_DATA); // illegal data
            return;
        }
        // guardo en variable global
        slave_address = value.u16;
        modbus_send_echo();
        break;
    case CONF_POS_BAUD_RATE:
        // check value
        if (!soft_uart_check_baud_rate((baud_rate_t)value.u16)){
            modbus_send_exception(MDOBUS_EXC_CODE_ILEGAL_DATA); // illegal data
            return;
        }
        modbus_send_echo();
        soft_uart_set_baud_rate(value.u16);
        break;
    case CONF_POS_PARITY:
        // check value
        if (value.u16 != MODBUS_DEFAULT_PARITY){
            modbus_send_exception(MDOBUS_EXC_CODE_ILEGAL_DATA); // illegal data
            return;
        }
        modbus_send_echo();
        break;
    case CONF_POS_STOP_BITS:
        // check value
        if (value.u16 != MODBUS_DEFAULT_STOP_BITS){
            modbus_send_exception(MDOBUS_EXC_CODE_ILEGAL_DATA); // illegal data
            return;
        }
        modbus_send_echo();
        break;
    default:
        modbus_send_exception(MDOBUS_EXC_CODE_ILEGAL_ADDRESS); // not implemented
        return;    
    }
    // guardar nuevo valor en el registro correspondiente
    if (eeprom_read_word(&config_registers[j]) != value.u16){
        eeprom_update_word(&config_registers[j], value.u16);
    }
}
#endif

static inline void modbus_process(void) {
    switch (request_buf[1]){
    case MODBUS_FUNCTION_READ_HOLDING_REGISTERS:
    case MODBUS_FUNCTION_READ_INPUT_REGISTERS:
        modbus_read_holding_registers();
        break;
#ifndef MODBUS_LITE_VERSION
    case MODBUS_FUNCTION_WRITE_SINGLE_REGISTER:
        modbus_write_holding_register();
        break;
#endif
    default:
        modbus_send_exception(MDOBUS_EXC_CODE_ILEGAL_FUNCTION); // illegal function
    }
}

void modbus_check_requests(void) {
    // comprobar si tengo datos entrantes
    while (soft_uart_available()){
        // guardar byte en buffer local
        request_buf[request_pos++] = soft_uart_read();
        // comprobar que el mensaje es para mi
        if (request_buf[0] == slave_address){
            // comprobar que tengo al menos 2 bytes leidos
            if (request_pos < 2) continue; // faltan bytes
            // procesar segun la funcion
            switch (request_buf[1]){
            case MODBUS_FUNCTION_READ_HOLDING_REGISTERS:
            case MODBUS_FUNCTION_READ_INPUT_REGISTERS:
            case MODBUS_FUNCTION_WRITE_SINGLE_REGISTER:
                // ambas funciones soportadas usan trama de tamaño fijo (8 bytes)
                if (request_pos < 8) continue; // faltan bytes
                // calcular y verificar crc
                crc16_init(&crc);
                crc16_step(&crc, request_buf, 6);
                if ((crc.u8[0] == request_buf[6]) && (crc.u8[1] == request_buf[7])){
                    // process the request
                    modbus_process();
                    // el buffer tiene EXACTAMENTE 8 bytes de capacidad, igual
                    // al tamaño de la trama recién consumida — no queda nada
                    // residual que conservar, alcanza con reiniciar la posición
                    request_pos = 0;
                    return;
                }
                // CRC inválido: se trata como trama corrupta, cae al
                // descarte de 1 byte de más abajo para resincronizar
                break;
            }
            // función desconocida (no matchea ningún case): también cae
            // al descarte de 1 byte para resincronizar, sin esperar más bytes
        }
        // la dirección no coincide, o la trama resultó inválida/desconocida:
        // descartar el byte más viejo y desplazar el resto para reintentar
        // sincronización a partir del siguiente byte
        request_pos--;
        for (uint8_t i = 0; i < request_pos; i++) {
            request_buf[i] = request_buf[i + 1];
        }
    }
}

void modbus_set_register(uint8_t pos, uint16_t *value){
    holding_registers[pos] = (uint16_t)*value;
}

void test_modbus_rtu(void){
    // modbus: enviar mensajes por USI UART
    uart_init(BAUD_RATE_9600);
    modbus_init();
    // loop principal
    while (1) {
        // check for MCU requests
        modbus_check_requests();
    }
}

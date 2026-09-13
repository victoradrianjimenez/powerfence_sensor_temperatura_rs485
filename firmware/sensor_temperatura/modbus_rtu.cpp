// docs: https://www.beijerelectronics.com/docs/DIO/GL-997X/en/modbus-interface.html

#include <avr/eeprom.h>
#include <util/delay.h>
#include "soft_uart.h"
#include "modbus_rtu.h"

typedef union {
    float f;
    uint16_t u16[2];
    uint32_t u32;
} my_data;

// Tamaño del buffer local
#define BUFFER_SIZE (8) // suficiente para almacenar el mensaje modbus más largo

static const uint8_t config_register_map[CONF_POS_COUNT] = {
    REG_ADDR_FIRMWARE_VERSION,
    REG_ADDR_SLAVE_ADDRESS,
    REG_ADDR_BAUD_RATE,
    REG_ADDR_PARITY,
    REG_ADDR_STOP_BITS
};

static const uint8_t var_register_map[VAR_COUNT] = {
    REG_TEMP, 
    REG_HUM
};

typedef union {
    uint16_t u16;
    uint8_t u8[2];
} my_uint16;

typedef union {
    uint32_t u32;
    uint8_t u8[4];
} my_uint32;

static uint8_t request_pos = 0;
static uint8_t request_buf[BUFFER_SIZE];
static uint16_t holding_registers[VAR_COUNT];
static uint16_t config_registers[CONF_POS_COUNT] EEMEM;   // en memoria no volatil
static my_uint16 crc;
static uint8_t slave_address;

static const uint16_t crc16_nibble_table[16] = {
    0x0000, 0xCC01, 0xD801, 0x1400,
    0xF001, 0x3C00, 0x2800, 0xE401,
    0xA001, 0x6C00, 0x7800, 0xB401,
    0x5000, 0x9C01, 0x8801, 0x4400
};

#define crc16_init(crc) ((crc)->u16 = 0xFFFF)

static void crc16_step(my_uint16* crc, uint8_t* d, uint8_t n) {
    for (uint8_t i=0; i<n; i++){
        crc->u16 ^= d[i];
        crc->u16 = (crc->u16 >> 4) ^ crc16_nibble_table[crc->u8[0] & 0x0F]; // nibble bajo
        crc->u16 = (crc->u16 >> 4) ^ crc16_nibble_table[crc->u8[0] & 0x0F]; // nibble alto
    }
}

void modbus_init(){
    // pongo datos en cero
    for (uint8_t i = 0; i < VAR_COUNT; i++){
        holding_registers[i] = 0;
    }
    // Si EEPROM nunca fue inicializada, poner valor por defecto
    slave_address = eeprom_read_word(&config_registers[CONF_POS_SLAVE_ADDRESS]);
    if (slave_address > 247 || slave_address == 0x00) {
        slave_address = MODBUS_DEFAULT_SLAVE_ADDRESS; // valor por defecto
        eeprom_update_word(&config_registers[CONF_POS_SLAVE_ADDRESS], MODBUS_DEFAULT_SLAVE_ADDRESS);
    }
    uint16_t baudrate = eeprom_read_word(&config_registers[CONF_POS_BAUD_RATE]);
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
}

static void modbus_send_exception(uint8_t exc_code) {
    uint8_t out[5] = {
        slave_address,
        request_buf[1] | 0x80,
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

static inline void modbus_send_echo(){
    soft_uart_de();
    soft_uart_send(request_buf, sizeof(request_buf));
    soft_uart_re();
}

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
        nregs << 1     // number of bytes
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

static inline void modbus_read_holding_registers(){
    my_uint16 addr = {.u8 = {request_buf[3], request_buf[2]}};
    uint8_t nregs = request_buf[5]; // uint16_t nregs = (request_buf[4] << 8) | request_buf[5];
    if (request_buf[4] != 0) {
        modbus_send_exception(MDOBUS_EXC_CODE_ILEGAL_DATA); //ilegal address
        return; // hasta 256 elementos
    }
    // verificar direccion
    if (!(addr.u16 >= REG_ADDR_BASE && addr.u16 + nregs <= REG_ADDR_BASE + VAR_COUNT)){
        if (!modbus_read_config_registers(addr.u16, nregs)){
            modbus_send_exception(MDOBUS_EXC_CODE_ILEGAL_ADDRESS); //ilegal address
        }
        return;
    }
    // preparo respuesta con registros
    crc16_init(&crc);
    my_uint16 val;
    uint8_t out[3] = {
        slave_address, // slave address
        MODBUS_FUNCTION_READ_HOLDING_REGISTERS, // command
        nregs << 1,    // number of bytes
    };
    crc16_step(&crc, out, 3);
    soft_uart_de();
    soft_uart_send(out, 3);
    while(nregs-- > 0){
        val.u16 = holding_registers[addr.u16++ - REG_ADDR_BASE];
        out[0] = val.u8[1];
        out[1] = val.u8[0];
        soft_uart_send(out, 2);
        crc16_step(&crc, out, 2);
    }
    soft_uart_send(crc.u8, 2);
    soft_uart_re();
}

static inline void modbus_write_holding_register(){
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
        if (!soft_uart_check_baud_rate(value.u16)){
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

static inline void modbus_process() {
    switch (request_buf[1]){
    case MODBUS_FUNCTION_READ_HOLDING_REGISTERS:
        modbus_read_holding_registers();
        break;
    case MODBUS_FUNCTION_WRITE_SINGLE_REGISTER:
        modbus_write_holding_register();
        break;
    default:
        modbus_send_exception(MDOBUS_EXC_CODE_ILEGAL_FUNCTION); // illegal function
    }
}

void modbus_check_requests() {
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
                }else{
                    soft_uart_send((uint8_t*)&crc, 2);
                    soft_uart_send(request_buf+6, 2);
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

void modbus_set_register(uint8_t pos, float *value){
    my_data var = {.f = *value};
    uint8_t reg = var_register_map[pos];
    holding_registers[reg] = var.u16[0];
    holding_registers[reg + 1] = var.u16[1];
}

void test_modbus_rtu(){
    // modbus: enviar mensajes por USI UART
    uart_init(BAUD_RATE_9600);
    modbus_init();
    // loop principal
    while (1) {
        // check for MCU requests
        modbus_check_requests();
    }
}

// docs: https://www.beijerelectronics.com/docs/DIO/GL-997X/en/modbus-interface.html

#include <avr/eeprom.h>
#include <util/delay.h>
#include "soft_uart.h"
#include "modbus_rtu.h"

// Tamaño del buffer local
#define BUFFER_SIZE (8) // suficiente para almacenar el mensaje modbus más largo

typedef union {
    uint16_t u16;
    uint8_t u8[2];
} my_uint16;

static uint8_t request_pos = 0;
static uint8_t request_buf[BUFFER_SIZE];
static uint16_t holding_registers[VAR_COUNT];
static uint16_t config_registers[CONF_POS_COUNT] EEMEM;   // en memoria no volatil

static uint8_t slave_address = MODBUS_DEFAULT_SLAVE_ADDRESS;

static my_uint16 crc16_init(){
    my_uint16 res = {.u16 = 0xFFFF};
    return res;
} 

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
}

static void modbus_send_exception(uint8_t exc_code) {
    uint8_t out[3] = {
        slave_address,
        (uint8_t)(request_buf[1] | 0x80),
        exc_code,
    };
    my_uint16 crc = crc16_init();
    crc16_step(&crc, out, 3);
    soft_uart_de();
    soft_uart_send(out, sizeof(out));
    soft_uart_send(crc.u8, 2);
    soft_uart_re();
}

static inline void modbus_send_echo(void){
    soft_uart_de();
    soft_uart_send(request_buf, sizeof(request_buf));
    soft_uart_re();
}

static inline void modbus_read_registers(uint16_t addr, uint8_t nregs, uint16_t base){
    uint8_t count = (base == 0) ? VAR_COUNT : CONF_POS_COUNT;
    // preparo respuesta con registros
    if (addr < base || (addr + nregs) > base + count){
        modbus_send_exception(MDOBUS_EXC_CODE_ILEGAL_ADDRESS); //ilegal address
        return;
    }
    // enviar respuesta con valor del registro
    my_uint16 crc = crc16_init();
    my_uint16 val;
    uint8_t out[3] = {
        slave_address, // slave address
        request_buf[1], // command
        (uint8_t)(nregs << 1)     // number of bytes
    };
    crc16_step(&crc, out, 3);
    soft_uart_de();
    soft_uart_send(out, 3);
    while(nregs-- > 0){
        if ((base == 0)){
            val.u16 = holding_registers[addr];
        }else{
            val.u16 = eeprom_read_word(&config_registers[addr-base]);
        }
        out[0] = val.u8[1]; //MSB
        out[1] = val.u8[0]; //LSB
        soft_uart_send(out, 2);
        crc16_step(&crc, out, 2);
        addr++;
    }
    soft_uart_send(crc.u8, 2);
    soft_uart_re();
}

static inline void modbus_write_holding_register(void){
    uint8_t j;
    my_uint16 addr = {.u8 = {request_buf[3], request_buf[2]}};
    // buscar posicion del registro en base a su address
    for (j = 0; j < CONF_POS_COUNT; j++){
        if (REG_ADDR_CONFIG_BASE + j == addr.u16) break;
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
        soft_uart_set_baud_rate((baud_rate_t)value.u16);
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

static inline void modbus_process(void) {
    switch (request_buf[1]){
    case MODBUS_FUNCTION_READ_HOLDING_REGISTERS: {
        my_uint16 addr = {.u8 = {request_buf[3], request_buf[2]}};
        my_uint16 nregs = {.u8 = {request_buf[5], request_buf[4]}};
        if (addr.u16 + nregs.u16 <= VAR_COUNT){
            modbus_read_registers(addr.u16, nregs.u16, 0);
        } else {
            modbus_read_registers(addr.u16, nregs.u16, REG_ADDR_CONFIG_BASE);
        }
        break;
    }
    case MODBUS_FUNCTION_WRITE_SINGLE_REGISTER:
        modbus_write_holding_register();
        break;
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
            case MODBUS_FUNCTION_WRITE_SINGLE_REGISTER:
                // ambas funciones soportadas usan trama de tamaño fijo (8 bytes)
                if (request_pos < 8) continue; // faltan bytes
                // calcular y verificar crc
                my_uint16 crc = crc16_init();
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

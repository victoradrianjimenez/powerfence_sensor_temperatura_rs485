// docs: https://www.beijerelectronics.com/docs/DIO/GL-997X/en/modbus-interface.html

#include <avr/eeprom.h>
#include <util/crc16.h>
#include "soft_uart.h"
#include "modbus_rtu.h"

// Tamaño del buffer local
#define BUFFER_SIZE (8) // suficiente para almacenar el mensaje modbus más largo
#define check_slave_address(addr)(addr > 0 && addr <= 247)
#define check_baud_rate(baud)(baud < BAUD_RATE_COUNT)

static uint8_t request_pos = 0;
static uint8_t request_buf[BUFFER_SIZE];
static uint16_t holding_registers[VAR_COUNT];
static uint16_t config_registers[CONF_POS_COUNT] EEMEM;   // en memoria no volatil
static uint8_t slave_address = MODBUS_DEFAULT_SLAVE_ADDRESS;
static uint16_t tx_crc; // CRC de la respuesta en curso

// big endian (wire) -> uint16
static inline uint16_t be16(const uint8_t *p) {
    return ((uint16_t)p[0] << 8) | p[1];
}

static inline uint16_t read_storage(uint8_t pos) {
    return eeprom_read_word(&config_registers[pos]);
}

static inline void write_storage(uint8_t pos, uint16_t val) {
    eeprom_update_word(&config_registers[pos], val);
}

void modbus_init(void) {
    // Si EEPROM nunca fue inicializada, poner valor por defecto
    uint8_t addr = read_storage(CONF_POS_SLAVE_ADDRESS);
    if (!check_slave_address(addr)) {
        addr = MODBUS_DEFAULT_SLAVE_ADDRESS;
        write_storage(CONF_POS_SLAVE_ADDRESS, addr);
    }
    slave_address = addr;
    write_storage(CONF_POS_BAUD_RATE, soft_uart_set_baud_rate((baud_rate_t)read_storage(CONF_POS_BAUD_RATE)));
}

static void tx_begin(void) {
    tx_crc = 0xFFFF;
    soft_uart_de();
}

static void tx_byte(uint8_t b) {
    tx_crc = _crc16_update(tx_crc, b); // CRC-16 Modbus (0xA001), en asm optimizado
    soft_uart_send(&b, 1);
}

static void tx_end(void) {
    soft_uart_send((uint8_t *)&tx_crc, 2); // AVR es little endian: LSB primero, como pide Modbus
    soft_uart_re();
}

static void modbus_send_exception(uint8_t exc_code) {
    tx_begin();
    tx_byte(slave_address);
    tx_byte(request_buf[1] | 0x80);
    tx_byte(exc_code);
    tx_end();
}

static inline void modbus_send_echo(void){
    soft_uart_de();
    soft_uart_send(request_buf, sizeof(request_buf));
    soft_uart_re();
}

static void modbus_read_registers(void) {
    uint16_t addr  = be16(&request_buf[2]);
    uint16_t nregs = be16(&request_buf[4]);
    uint16_t off   = addr - REG_ADDR_REG_BASE;
    uint16_t count = VAR_COUNT;
    uint8_t  cfg   = 0;

    if (off >= VAR_COUNT) { // fuera de la tabla de datos -> tabla de config
        off   = addr - REG_ADDR_CONFIG_BASE;
        count = CONF_POS_COUNT;
        cfg   = 1;
    }
    // comparación sin overflow (addr + nregs podía dar la vuelta en 16 bits)
    if (off >= count || nregs > count - off) {
        modbus_send_exception(MODBUS_EXC_CODE_ILEGAL_ADDRESS);
        return;
    }

    uint8_t n = (uint8_t)nregs;
    tx_begin();
    tx_byte(slave_address);
    tx_byte(request_buf[1]);
    tx_byte(n << 1);
    while (n--) {
        uint16_t v = cfg ? read_storage(off) : holding_registers[off];
        off++;
        tx_byte(v >> 8);
        tx_byte((uint8_t)v);
    }
    tx_end();
}

static void modbus_write_holding_register(void) {
    uint16_t value = be16(&request_buf[4]);
    // sin bucle de búsqueda: el switch resuelve directo por dirección
    switch (be16(&request_buf[2]) - REG_ADDR_CONFIG_BASE) {
    case CONF_POS_SLAVE_ADDRESS:
        if (!check_slave_address(value)) goto bad_data;
        slave_address = value;
        modbus_send_echo();
        write_storage(CONF_POS_SLAVE_ADDRESS, value);
        return;
    case CONF_POS_BAUD_RATE:
        if (!check_baud_rate(value)) goto bad_data;
        modbus_send_echo();
        write_storage(CONF_POS_BAUD_RATE, soft_uart_set_baud_rate((baud_rate_t)value));
        return;
    default:
        modbus_send_exception(MODBUS_EXC_CODE_ILEGAL_ADDRESS);
        return;
    }
bad_data:
    modbus_send_exception(MODBUS_EXC_CODE_ILEGAL_DATA);
}

void modbus_check_requests(void) {
    uint8_t fn;
    uint16_t crc;
    // comprobar si tengo datos entrantes
    if (!soft_uart_available()) return; // debo esperar datos
    // guardar byte en buffer local
    request_buf[request_pos++] = soft_uart_read();
    // comprobar que el mensaje es para mi
    if (request_buf[0] != slave_address) goto err;
    // comprobar que tengo al menos 2 bytes leidos
    if (request_pos < 2) return; // debo esperar resto de bytes
    // comprobar la funcion. Si es desconocida, cae al descarte de 1 byte para resincronizar
    fn = request_buf[1];
    if (fn != MODBUS_FUNCTION_READ_HOLDING_REGISTERS && fn != MODBUS_FUNCTION_WRITE_SINGLE_REGISTER) goto err;
    // comprobar si tengo mensaje completo. Las funciones soportadas usan trama de tamaño fijo (8 bytes)
    if (request_pos < BUFFER_SIZE) return; // debo esperar resto de bytes
    // calcular y verificar crc
    crc = 0xFFFF;
    for (uint8_t i = 0; i < BUFFER_SIZE; i++) {
        crc = _crc16_update(crc, request_buf[i]);
    }
    if (crc != 0) goto err; // CRC inválido: se trata como trama corrupta
    // process the request
    if (fn == MODBUS_FUNCTION_READ_HOLDING_REGISTERS) {
        modbus_read_registers();
    } else {
        modbus_write_holding_register();
    }
    request_pos = 0;
    return;
err:
    // la dirección no coincide, o la trama resultó inválida/desconocida: descartar el byte más viejo y desplazar el resto para reintentar sincronización a partir del siguiente byte
    request_pos--;
    for (uint8_t i = 0; i < request_pos; i++) {
        request_buf[i] = request_buf[i + 1];
    }
}

void modbus_set_register(modbus_register_position_t pos, uint16_t value){
    if (pos < VAR_COUNT) holding_registers[pos] = value;
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

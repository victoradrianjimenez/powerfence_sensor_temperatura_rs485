#pragma once
#include <stdint.h>

#define FIRMWARE_MAYOR_VERSION      1
#define FIRMWARE_MINOR_VERSION      0

/*****************************************************************************/

#define REG_ADDR_CONFIG_BASE                   100
#define REG_ADDR_CONFIG_FIRMWARE_VERSION       0
#define REG_ADDR_CONFIG_SLAVE_ADDRESS          1
#define REG_ADDR_CONFIG_BAUD_RATE              2
#define REG_ADDR_CONFIG_PARITY                 3
#define REG_ADDR_CONFIG_STOP_BITS              4

enum {
    CONF_POS_FIRMWARE_VERSION=0,
    CONF_POS_SLAVE_ADDRESS,
    CONF_POS_BAUD_RATE,
    CONF_POS_PARITY,
    CONF_POS_STOP_BITS,
    CONF_POS_COUNT,
};

/*****************************************************************************/

enum {
    VAR_TEMP=0,
    VAR_HUM,
    VAR_COUNT
};

/*****************************************************************************/

#define MODBUS_DEFAULT_SLAVE_ADDRESS 0x01
#define MODBUS_DEFAULT_BAUD_RATE 9600
#define MODBUS_DEFAULT_PARITY 0
#define MODBUS_DEFAULT_STOP_BITS 1
#define FIRMWARE_VERSION ((FIRMWARE_MAYOR_VERSION << 8) | FIRMWARE_MINOR_VERSION)

#define MODBUS_FUNCTION_READ_HOLDING_REGISTERS 0x03
#define MODBUS_FUNCTION_READ_INPUT_REGISTERS   0x04
#define MODBUS_FUNCTION_WRITE_SINGLE_REGISTER  0x06

#define MDOBUS_EXC_CODE_ILEGAL_FUNCTION 0x01
#define MDOBUS_EXC_CODE_ILEGAL_ADDRESS  0x02
#define MDOBUS_EXC_CODE_ILEGAL_DATA     0x03
#define MDOBUS_EXC_CODE_SLAVE_ERROR     0x04
#define MDOBUS_EXC_CODE_SLAVE_BUSY      0x06

/*****************************************************************************/

/**
 * @brief Inicializar módulo poniendo en cero los registros y cargando configuración.
 */
void modbus_init(void);

/**
 * @brief Comprobar si tengo solicituders y procesarlas en caso de tenerlas.
 */
void modbus_check_requests(void);

/**
 * @brief Establecer el valor de un registro.
 * @param pos Posición dentro del array de variables (no es la dirección modbus).
 * @param value Nuevo valor para el registro.
 */
void modbus_set_register(uint8_t pos, uint16_t *value);

/**
 * @brief Funcion de prueba.
 */
void test_modbus_rtu(void);

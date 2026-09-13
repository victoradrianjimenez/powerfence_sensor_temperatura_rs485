#pragma once

#define FIRMWARE_MAYOR_VERSION      1
#define FIRMWARE_MINOR_VERSION      0

/*****************************************************************************/

#define REG_ADDR_BASE                   0

#define REG_TEMP                        1 //Int16
#define REG_HUM                         2 //Int16

#define REG_ADDR_FIRMWARE_VERSION       100
#define REG_ADDR_SLAVE_ADDRESS          101
#define REG_ADDR_BAUD_RATE              102
#define REG_ADDR_PARITY                 103
#define REG_ADDR_STOP_BITS              104

enum {
    CONF_POS_FIRMWARE_VERSION=0,
    CONF_POS_SLAVE_ADDRESS,
    CONF_POS_BAUD_RATE,
    CONF_POS_PARITY,
    CONF_POS_STOP_BITS,
    CONF_POS_COUNT,
} conf_variable_position;

/*****************************************************************************/

enum {
    VAR_TEMP=0,
    VAR_HUM,
    VAR_COUNT
} variable_position;

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
void modbus_init();

/**
 * @brief Comprobar si tengo solicituders y procesarlas en caso de tenerlas.
 */
void modbus_check_requests();

/**
 * @brief Establecer el valor de un registro.
 * @param pos Posición dentro del array de variables (no es la dirección modbus).
 * @param value Nuevo valor para el registro.
 */
void modbus_set_register(uint8_t pos, float *value);

/**
 * @brief Funcion de prueba con valores fijos para los registros.
 */
void test_modbus_rtu();

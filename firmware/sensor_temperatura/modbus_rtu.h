#pragma once
#include <stdint.h>

#define REG_ADDR_CONFIG_BASE 100
enum {
    CONF_POS_SLAVE_ADDRESS=0,
    CONF_POS_BAUD_RATE,
    CONF_POS_COUNT,
};

#define REG_ADDR_REG_BASE 0
typedef enum {
    VAR_TEMP=0,
    VAR_HUM,
    VAR_COUNT
} modbus_register_position_t;

#define MODBUS_DEFAULT_SLAVE_ADDRESS 0x01

#define MODBUS_FUNCTION_READ_HOLDING_REGISTERS 0x03
#define MODBUS_FUNCTION_WRITE_SINGLE_REGISTER  0x06
//#define MODBUS_FUNCTION_READ_INPUT_REGISTERS   0x04

#define MODBUS_EXC_CODE_ILEGAL_ADDRESS  0x02
#define MODBUS_EXC_CODE_ILEGAL_DATA     0x03
//#define MODBUS_EXC_CODE_ILEGAL_FUNCTION 0x01
//#define MODBUS_EXC_CODE_SLAVE_ERROR     0x04
//#define MODBUS_EXC_CODE_SLAVE_BUSY      0x06

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
void modbus_set_register(modbus_register_position_t pos, uint16_t value);

/**
 * @brief Funcion de prueba.
 */
void test_modbus_rtu(void);

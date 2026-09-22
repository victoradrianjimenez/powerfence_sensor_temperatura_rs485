/*
  Lectura del HTU21D con Arduino Nano usando los comandos "no hold master"
  (0xF3 temperatura, 0xF5 humedad), sin librerias externas.

  Conexiones:
    VCC -> 3.3V
    GND -> GND
    SDA -> A4
    SCL -> A5

  Funcionamiento:
    1. Se envia el comando de medicion y se libera el bus.
    2. Se intenta leer 3 bytes (MSB, LSB, CRC). Mientras el sensor mide,
       responde NACK a la direccion de lectura y requestFrom() devuelve 0.
    3. Se reintenta hasta recibir los 3 bytes o agotar el tiempo.
*/

#include <Wire.h>

#define HTU21D_ADDR       0x40
#define CMD_TEMP_NOHOLD   0xF3
#define CMD_HUM_NOHOLD    0xF5
#define CMD_SOFT_RESET    0xFE

#define TIMEOUT_MS        100   // maximo real: ~50 ms (T 14 bits)
#define POLL_MS           5     // intervalo entre reintentos

enum htu_status_t {
  HTU_OK = 0,
  HTU_ERR_CMD,       // el sensor no respondio al comando
  HTU_ERR_TIMEOUT,   // la medicion no termino a tiempo
  HTU_ERR_CRC,       // CRC incorrecto
  HTU_ERR_TYPE       // el bit de estado no coincide con la medicion pedida
};

// CRC-8, polinomio x^8 + x^5 + x^4 + 1 (0x131)
uint8_t htuCRC(uint8_t msb, uint8_t lsb) {
  uint8_t crc = 0;
  uint8_t datos[2] = { msb, lsb };
  for (uint8_t i = 0; i < 2; i++) {
    crc ^= datos[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x80) crc = (crc << 1) ^ 0x31;
      else            crc <<= 1;
    }
  }
  return crc;
}

// Lanza una medicion no hold y devuelve el valor crudo (sin bits de estado)
htu_status_t htuMedir(uint8_t comando, uint16_t &raw) {
  Wire.beginTransmission(HTU21D_ADDR);
  Wire.write(comando);
  if (Wire.endTransmission() != 0) return HTU_ERR_CMD;

  unsigned long inicio = millis();
  while (millis() - inicio < TIMEOUT_MS) {
    delay(POLL_MS);

    // Si el sensor sigue midiendo responde NACK y devuelve 0
    if (Wire.requestFrom((uint8_t)HTU21D_ADDR, (uint8_t)3) != 3) {
      while (Wire.available()) Wire.read();   // vaciar por seguridad
      continue;
    }

    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    uint8_t crc = Wire.read();

    if (htuCRC(msb, lsb) != crc){
      // Depuracion: bytes recibidos y CRC esperado
      Serial.print(F("[CRC] rx: "));
      Serial.print(msb, HEX);  Serial.print(' ');
      Serial.print(lsb, HEX);  Serial.print(' ');
      Serial.print(crc, HEX);
      Serial.print(F("  esperado: "));
      Serial.println(htuCRC(msb, lsb), HEX);
      return HTU_ERR_CRC;
    }


    // Bit 1 de LSB: 0 = temperatura, 1 = humedad
    bool esHumedad = (lsb & 0x02) != 0;
    if (esHumedad != (comando == CMD_HUM_NOHOLD)) return HTU_ERR_TYPE;

    raw = ((uint16_t)msb << 8) | (lsb & 0xFC);  // borrar bits de estado
    return HTU_OK;
  }
  return HTU_ERR_TIMEOUT;
}

htu_status_t htuLeerTemperatura(float &temp) {
  uint16_t raw;
  htu_status_t st = htuMedir(CMD_TEMP_NOHOLD, raw);
  if (st == HTU_OK) temp = -46.85f + 175.72f * raw / 65536.0f;
  return st;
}

htu_status_t htuLeerHumedad(float &hum) {
  uint16_t raw;
  htu_status_t st = htuMedir(CMD_HUM_NOHOLD, raw);
  if (st == HTU_OK) hum = -6.0f + 125.0f * raw / 65536.0f;
  return st;
}

void imprimirError(htu_status_t st) {
  switch (st) {
    case HTU_ERR_CMD:     Serial.print(F("sin respuesta al comando")); break;
    case HTU_ERR_TIMEOUT: Serial.print(F("timeout de medicion"));      break;
    case HTU_ERR_CRC:     Serial.print(F("error de CRC"));             break;
    case HTU_ERR_TYPE:    Serial.print(F("tipo de dato incorrecto"));  break;
    default: break;
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Soft reset y espera de arranque (max 15 ms segun datasheet)
  Wire.beginTransmission(HTU21D_ADDR);
  Wire.write(CMD_SOFT_RESET);
  if (Wire.endTransmission() != 0) {
    Serial.println(F("No se encontro el HTU21D. Revisa las conexiones."));
    while (true);
  }
  delay(20);

  Serial.println(F("HTU21D listo (modo no hold)."));
}

void loop() {
  float temperatura, humedad;

  htu_status_t stT = htuLeerTemperatura(temperatura);
  htu_status_t stH = htuLeerHumedad(humedad);

  Serial.print(F("Temperatura: "));
  if (stT == HTU_OK) { Serial.print(temperatura, 1); Serial.print(F(" C")); }
  else               { imprimirError(stT); }

  Serial.print(F("\tHumedad: "));
  if (stH == HTU_OK) { Serial.print(humedad, 1); Serial.println(F(" %")); }
  else               { imprimirError(stH); Serial.println(); }

  delay(2000);
}

#include <SoftwareSerial.h>
#include <ModbusMaster.h>

#define SLAVE_ADDRESS 0x01
#define RE_DE 2
SoftwareSerial rs485(10, 11); // RX, TX
ModbusMaster master;

String inputString = "";
bool stringComplete = false;

uint16_t crc16(uint8_t *buf, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t pos = 0; pos < len; pos++) {
    crc ^= buf[pos];
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 1)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }
  return crc;
}

// Control RS485
void preTransmission() {
  digitalWrite(RE_DE, HIGH);
}

void postTransmission() {
  digitalWrite(RE_DE, LOW);
}

void setup() {
  pinMode(RE_DE, OUTPUT);
  digitalWrite(RE_DE, LOW);
  Serial.begin(9600);
  rs485.begin(9600); // 2400, 4800, 9600, 14400, 19200, 31250
  master.begin(SLAVE_ADDRESS, rs485);
  master.preTransmission(preTransmission);
  master.postTransmission(postTransmission);
  inputString.reserve(50);
}

// Captura lo que escribes por consola
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n' || inChar == '\r') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}

// Función para convertir dos uint16 a float (Big-Endian / Palabra alta primero)
float wordsToFloat(uint16_t highWord, uint16_t lowWord) {
    // Combinamos las dos palabras de 16 bits en un valor de 32 bits
    uint32_t combined = ((uint32_t)highWord << 16) | lowWord;
    
    // Usamos una unión para reinterpretar los bits directamente como float
    union {
        uint32_t i;
        float f;
    } converter;
    
    converter.i = combined;
    return converter.f;
}

// Procesa un comando ingreado por puerto serie
void processCommand(String cmd) {
  cmd.trim();

  // --- MODO ESCRITURA: "1000=123" ---
  if (cmd.indexOf('=') > 0) {
    int sep = cmd.indexOf('=');
    int reg = cmd.substring(0, sep).toInt();
    int value = cmd.substring(sep + 1).toInt();

    Serial.print("Escribiendo REG ");
    Serial.print(reg);
    Serial.print(" = ");
    Serial.println(value);

    uint8_t result = master.writeSingleRegister(reg, value);
    if (result == master.ku8MBSuccess)
      Serial.println("OK");
    else {
      Serial.print("Error Modbus: ");
      Serial.println(result);
    }
    return;
  }

  // --- MODO LECTURA MULTIPLE: "1000:1010" ---
  else if (cmd.indexOf(':') > 0) {
    int sep = cmd.indexOf(':');
    int startReg = cmd.substring(0, sep).toInt();
    int endReg = cmd.substring(sep + 1).toInt();
    int count = endReg - startReg + 1;
    if (count <= 0) {
      Serial.println("Rango invalido.");
      return;
    }

    Serial.print("Leyendo ");
    Serial.print(count);
    Serial.print(" registros desde ");
    Serial.println(startReg);

    uint8_t result = master.readHoldingRegisters(startReg, count);
    if (result == master.ku8MBSuccess) {
      for (int i = 0; i < count; i++) {
        Serial.print("Reg ");
        Serial.print(startReg + i);
        Serial.print(" = ");
        Serial.println(master.getResponseBuffer(i));
      }
    } else {
      Serial.print("Error Modbus: ");
      Serial.println(result);
    }
    return;
  }

  Serial.println("Comando no reconocido.");
}

void loop() {
  if (stringComplete) {
    processCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
}

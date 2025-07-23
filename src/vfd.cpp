#include "vfd.h"

VFD::VFD(ModbusMaster* node, uint8_t address)
  : node(node), address(address) {
  node->begin(address, Serial2);
}

// ------------------------------------------------------------------
//  Generic helpers – used by ALL subclasses
// ------------------------------------------------------------------
bool VFD::readModbusRegister(uint16_t reg, uint16_t* value) {
  unsigned long startTime = millis();
  uint8_t result;
  uint8_t attempts = 0;

  do {
    attempts++;
    result = node->readHoldingRegisters(reg, 1);

    if (result == node->ku8MBSuccess) {
      *value = node->getResponseBuffer(0);
      return true;
    }

    delay(50);
  } while (millis() - startTime < MODBUS_RESPONSE_TIMEOUT);

  return false;
}

bool VFD::writeModbusRegister(uint16_t reg, uint16_t value) {
  unsigned long startTime = millis();
  uint8_t result;

  do {
    result = node->writeSingleRegister(reg, value);
    if (result == node->ku8MBSuccess) {
      return true;
    }
    delay(50);
  } while (millis() - startTime < MODBUS_RESPONSE_TIMEOUT);

  return false;
}

bool VFD::verifyModbusConnection() {
  static uint32_t lastCheck = 0;
  static bool lastStatus = false;

  if (millis() - lastCheck < MODBUS_RESPONSE_TIMEOUT) return lastStatus;

  lastCheck = millis();
  uint16_t value;
  lastStatus = readModbusRegister(MB_STATUS_REG, &value);

  if (!lastStatus) {
    node->begin(address, Serial2);
  }

  return lastStatus;
}

void VFD::checkModbusAddress() {
  uint16_t value;
  if (readModbusRegister(6001, &value)) {
    Serial.print("Endereço MODBUS atual: ");
    Serial.println(value);
  }
}

bool VFD::resetFault() {
  return sendCommandWithRetry(CMD_RESET_FAULT);
}

// ---------- Schneider ATV12 ----------
SchneiderATV12::SchneiderATV12(ModbusMaster* node, uint8_t address)
  : VFD(node, address) {
  MB_CMD_REG = 8501;
  MB_SPEED_REG = 8602;
  MB_ACTUAL_SPEED_REG = 8604;
  MB_STATUS_REG = 3201;
  MB_ERRORS_REG = 7200;
  MB_MAX_FREQ_REG = 3103;
  MB_MIN_FREQ_REG = 3105;
  MB_HIGH_SPEED_REG = 3104;
}

bool SchneiderATV12::setMotorSpeed(uint16_t speed) {
  return writeModbusRegister(MB_SPEED_REG, speed);
}

bool SchneiderATV12::setMotorDirection(uint16_t direction) {
  uint16_t currentCmd;
  if (!readModbusRegister(MB_CMD_REG, &currentCmd)) return false;

  if (direction) {
    currentCmd |= (1 << 11);
  } else {
    currentCmd &= ~(1 << 11);
  }

  return writeModbusRegister(MB_CMD_REG, currentCmd);
}

bool SchneiderATV12::sendCommandWithRetry(uint16_t command) {
  for (uint8_t i = 0; i < 3; i++) {
    if (writeModbusRegister(MB_CMD_REG, command)) {
      return true;
    }
    delay(100);
  }
  return false;
}

void SchneiderATV12::readFrequencyLimits() {
  static unsigned long lastCheck = 0;
  const unsigned long checkInterval = 30000;

  if (millis() - lastCheck > checkInterval) {
    uint16_t newMax, newMin, newHigh;
    bool success = true;

    success &= readModbusRegister(MB_MAX_FREQ_REG, &newMax);
    success &= readModbusRegister(MB_MIN_FREQ_REG, &newMin);
    success &= readModbusRegister(MB_HIGH_SPEED_REG, &newHigh);

    if (success && newMin < newMax && newHigh <= newMax) {
      minFrequency = newMin;
      maxFrequency = newMax;
      highSpeed = newHigh;
    }

    lastCheck = millis();
  }
}

void SchneiderATV12::readMotorStatus() {
  uint16_t value;

  if (readModbusRegister(MB_STATUS_REG, &value)) {
    motorStatus = value;
  }

  if (readModbusRegister(MB_ACTUAL_SPEED_REG, &value)) {
    motorSpeed = value;
  }

  if (readModbusRegister(MB_ERRORS_REG, &value)) {
    motorErrors = value;
  }

  motorDirection = (motorStatus & 0x8000) ? 1 : 0;
}

// ---------- Siemens V20 ----------
SiemensV20::SiemensV20(ModbusMaster* node, uint8_t address)
  : VFD(node, address) {
  MB_CMD_REG = 0x047E;
  MB_SPEED_REG = 0x047F;
  MB_ACTUAL_SPEED_REG = 0x0481;
  MB_STATUS_REG = 0x0402;
  MB_ERRORS_REG = 0x0500;
  MB_MAX_FREQ_REG = 0x0600;
  MB_MIN_FREQ_REG = 0x0601;
}

bool SiemensV20::setMotorSpeed(uint16_t speed) {
  return writeModbusRegister(MB_SPEED_REG, speed);
}

bool SiemensV20::setMotorDirection(uint16_t direction) {
  uint16_t currentCmd;
  if (!readModbusRegister(MB_CMD_REG, &currentCmd)) return false;

  if (direction) {
    currentCmd |= (1 << 11);
  } else {
    currentCmd &= ~(1 << 11);
  }

  return writeModbusRegister(MB_CMD_REG, currentCmd);
}

bool SiemensV20::sendCommandWithRetry(uint16_t command) {
  for (uint8_t i = 0; i < 3; i++) {
    if (writeModbusRegister(MB_CMD_REG, command)) {
      return true;
    }
    delay(100);
  }
  return false;
}

void SiemensV20::readFrequencyLimits() {
  static unsigned long lastCheck = 0;
  const unsigned long checkInterval = 30000;

  if (millis() - lastCheck > checkInterval) {
    uint16_t newMax, newMin;
    bool success = true;

    success &= readModbusRegister(MB_MAX_FREQ_REG, &newMax);
    success &= readModbusRegister(MB_MIN_FREQ_REG, &newMin);

    if (success && newMin < newMax) {
      minFrequency = newMin;
      maxFrequency = newMax;
    }

    lastCheck = millis();
  }
}

void SiemensV20::readMotorStatus() {
  uint16_t value;

  if (readModbusRegister(MB_STATUS_REG, &value)) {
    motorStatus = value;
  }

  if (readModbusRegister(MB_ACTUAL_SPEED_REG, &value)) {
    motorSpeed = value;
  }

  if (readModbusRegister(MB_ERRORS_REG, &value)) {
    motorErrors = value;
  }

  motorDirection = (motorStatus & 0x8000) ? 1 : 0;
}

// ---------- WEG CFW500 ----------
WEGCFW500::WEGCFW500(ModbusMaster* node, uint8_t address)
  : VFD(node, address) {
  MB_CMD_REG = 0x2000;
  MB_SPEED_REG = 0x2001;
  MB_ACTUAL_SPEED_REG = 0x3001;
  MB_STATUS_REG = 0x1000;
  MB_ERRORS_REG = 0x1001;
  MB_MAX_FREQ_REG = 0x3002;
  MB_MIN_FREQ_REG = 0x3003;
}

bool WEGCFW500::setMotorSpeed(uint16_t speed) {
  return writeModbusRegister(MB_SPEED_REG, speed);
}

bool WEGCFW500::setMotorDirection(uint16_t direction) {
  uint16_t currentCmd;
  if (!readModbusRegister(MB_CMD_REG, &currentCmd)) return false;

  if (direction) {
    currentCmd |= (1 << 11);
  } else {
    currentCmd &= ~(1 << 11);
  }

  return writeModbusRegister(MB_CMD_REG, currentCmd);
}

bool WEGCFW500::sendCommandWithRetry(uint16_t command) {
  for (uint8_t i = 0; i < 3; i++) {
    if (writeModbusRegister(MB_CMD_REG, command)) {
      return true;
    }
    delay(100);
  }
  return false;
}

void WEGCFW500::readFrequencyLimits() {
  static unsigned long lastCheck = 0;
  const unsigned long checkInterval = 30000;

  if (millis() - lastCheck > checkInterval) {
    uint16_t newMax, newMin;
    bool success = true;

    success &= readModbusRegister(MB_MAX_FREQ_REG, &newMax);
    success &= readModbusRegister(MB_MIN_FREQ_REG, &newMin);

    if (success && newMin < newMax) {
      minFrequency = newMin;
      maxFrequency = newMax;
    }

    lastCheck = millis();
  }
}

void WEGCFW500::readMotorStatus() {
  uint16_t value;

  if (readModbusRegister(MB_STATUS_REG, &value)) {
    motorStatus = value;
  }

  if (readModbusRegister(MB_ACTUAL_SPEED_REG, &value)) {
    motorSpeed = value;
  }

  if (readModbusRegister(MB_ERRORS_REG, &value)) {
    motorErrors = value;
  }

  motorDirection = (motorStatus & 0x8000) ? 1 : 0;
}

// ---------- ABB ACS550 ----------
ABBACS550::ABBACS550(ModbusMaster* node, uint8_t address)
  : VFD(node, address) {
  MB_CMD_REG = 0x2000;
  MB_SPEED_REG = 0x2001;
  MB_ACTUAL_SPEED_REG = 0x2002;
  MB_STATUS_REG = 0x2100;
  MB_ERRORS_REG = 0x2200;
  MB_MAX_FREQ_REG = 0x2300;
  MB_MIN_FREQ_REG = 0x2301;
}

bool ABBACS550::setMotorSpeed(uint16_t speed) {
  return writeModbusRegister(MB_SPEED_REG, speed);
}

bool ABBACS550::setMotorDirection(uint16_t direction) {
  uint16_t currentCmd;
  if (!readModbusRegister(MB_CMD_REG, &currentCmd)) return false;

  if (direction) {
    currentCmd |= (1 << 11);
  } else {
    currentCmd &= ~(1 << 11);
  }

  return writeModbusRegister(MB_CMD_REG, currentCmd);
}

bool ABBACS550::sendCommandWithRetry(uint16_t command) {
  for (uint8_t i = 0; i < 3; i++) {
    if (writeModbusRegister(MB_CMD_REG, command)) {
      return true;
    }
    delay(100);
  }
  return false;
}

void ABBACS550::readFrequencyLimits() {
  static unsigned long lastCheck = 0;
  const unsigned long checkInterval = 30000;

  if (millis() - lastCheck > checkInterval) {
    uint16_t newMax, newMin;
    bool success = true;

    success &= readModbusRegister(MB_MAX_FREQ_REG, &newMax);
    success &= readModbusRegister(MB_MIN_FREQ_REG, &newMin);

    if (success && newMin < newMax) {
      minFrequency = newMin;
      maxFrequency = newMax;
    }

    lastCheck = millis();
  }
}

void ABBACS550::readMotorStatus() {
  uint16_t value;

  if (readModbusRegister(MB_STATUS_REG, &value)) {
    motorStatus = value;
  }

  if (readModbusRegister(MB_ACTUAL_SPEED_REG, &value)) {
    motorSpeed = value;
  }

  if (readModbusRegister(MB_ERRORS_REG, &value)) {
    motorErrors = value;
  }

  motorDirection = (motorStatus & 0x8000) ? 1 : 0;
}

// ---------- Danfoss FC302 ----------
DanfossFC302::DanfossFC302(ModbusMaster* node, uint8_t address)
  : VFD(node, address) {
  MB_CMD_REG = 0x2000;
  MB_SPEED_REG = 0x2001;
  MB_ACTUAL_SPEED_REG = 0x2002;
  MB_STATUS_REG = 0x3000;
  MB_ERRORS_REG = 0x3001;
  MB_MAX_FREQ_REG = 0x3002;
  MB_MIN_FREQ_REG = 0x3003;
}

bool DanfossFC302::setMotorSpeed(uint16_t speed) {
  return writeModbusRegister(MB_SPEED_REG, speed);
}

bool DanfossFC302::setMotorDirection(uint16_t direction) {
  uint16_t currentCmd;
  if (!readModbusRegister(MB_CMD_REG, &currentCmd)) return false;

  if (direction) {
    currentCmd |= (1 << 11);
  } else {
    currentCmd &= ~(1 << 11);
  }

  return writeModbusRegister(MB_CMD_REG, currentCmd);
}

bool DanfossFC302::sendCommandWithRetry(uint16_t command) {
  for (uint8_t i = 0; i < 3; i++) {
    if (writeModbusRegister(MB_CMD_REG, command)) {
      return true;
    }
    delay(100);
  }
  return false;
}

void DanfossFC302::readFrequencyLimits() {
  static unsigned long lastCheck = 0;
  const unsigned long checkInterval = 30000;

  if (millis() - lastCheck > checkInterval) {
    uint16_t newMax, newMin;
    bool success = true;

    success &= readModbusRegister(MB_MAX_FREQ_REG, &newMax);
    success &= readModbusRegister(MB_MIN_FREQ_REG, &newMin);

    if (success && newMin < newMax) {
      minFrequency = newMin;
      maxFrequency = newMax;
    }

    lastCheck = millis();
  }
}

void DanfossFC302::readMotorStatus() {
  uint16_t value;

  if (readModbusRegister(MB_STATUS_REG, &value)) {
    motorStatus = value;
  }

  if (readModbusRegister(MB_ACTUAL_SPEED_REG, &value)) {
    motorSpeed = value;
  }

  if (readModbusRegister(MB_ERRORS_REG, &value)) {
    motorErrors = value;
  }

  motorDirection = (motorStatus & 0x8000) ? 1 : 0;
}

// ---------- Yaskawa V1000 ----------
YaskawaV1000::YaskawaV1000(ModbusMaster* node, uint8_t address)
  : VFD(node, address) {
  MB_CMD_REG = 0x0400;
  MB_SPEED_REG = 0x0401;
  MB_ACTUAL_SPEED_REG = 0x0402;
  MB_STATUS_REG = 0x0500;
  MB_ERRORS_REG = 0x0501;
  MB_MAX_FREQ_REG = 0x0600;
  MB_MIN_FREQ_REG = 0x0601;
}

bool YaskawaV1000::setMotorSpeed(uint16_t speed) {
  return writeModbusRegister(MB_SPEED_REG, speed);
}

bool YaskawaV1000::setMotorDirection(uint16_t direction) {
  uint16_t currentCmd;
  if (!readModbusRegister(MB_CMD_REG, &currentCmd)) return false;

  if (direction) {
    currentCmd |= (1 << 11);
  } else {
    currentCmd &= ~(1 << 11);
  }

  return writeModbusRegister(MB_CMD_REG, currentCmd);
}

bool YaskawaV1000::sendCommandWithRetry(uint16_t command) {
  for (uint8_t i = 0; i < 3; i++) {
    if (writeModbusRegister(MB_CMD_REG, command)) {
      return true;
    }
    delay(100);
  }
  return false;
}

void YaskawaV1000::readFrequencyLimits() {
  static unsigned long lastCheck = 0;
  const unsigned long checkInterval = 30000;

  if (millis() - lastCheck > checkInterval) {
    uint16_t newMax, newMin;
    bool success = true;

    success &= readModbusRegister(MB_MAX_FREQ_REG, &newMax);
    success &= readModbusRegister(MB_MIN_FREQ_REG, &newMin);

    if (success && newMin < newMax) {
      minFrequency = newMin;
      maxFrequency = newMax;
    }

    lastCheck = millis();
  }
}

void YaskawaV1000::readMotorStatus() {
  uint16_t value;

  if (readModbusRegister(MB_STATUS_REG, &value)) {
    motorStatus = value;
  }

  if (readModbusRegister(MB_ACTUAL_SPEED_REG, &value)) {
    motorSpeed = value;
  }

  if (readModbusRegister(MB_ERRORS_REG, &value)) {
    motorErrors = value;
  }

  motorDirection = (motorStatus & 0x8000) ? 1 : 0;
}

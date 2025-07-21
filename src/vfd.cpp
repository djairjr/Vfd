#include "vfd.h"

VFD::VFD(ModbusMaster* node, uint8_t address)
  : node(node), address(address) {
  node->begin(address, Serial2);
}

// ------------------------------------------------------------------
//  Generic helpers – used by ALL subclasses
// ------------------------------------------------------------------
bool VFD::readModbusRegister(uint16_t reg, uint16_t* value) {
  unsigned long start = millis();
  uint8_t res;
  uint8_t att = 0;
  do {
    res = node->readHoldingRegisters(reg, 1);
    if (res == node->ku8MBSuccess) {
      *value = node->getResponseBuffer(0);
      return true;
    }
    delay(50);
  } while (millis() - start < 2000);   // MODBUS_RESPONSE_TIMEOUT
  return false;
}

bool VFD::writeModbusRegister(uint16_t reg, uint16_t value) {
  unsigned long start = millis();
  uint8_t res;
  do {
    res = node->writeSingleRegister(reg, value);
    if (res == node->ku8MBSuccess) return true;
    delay(50);
  } while (millis() - start < 2000);
  return false;
}

bool VFD::verifyModbusConnection() {
  static uint32_t lastCheck = 0;
  static bool lastStatus = false;
  if (millis() - lastCheck < 2000) return lastStatus;

  lastCheck = millis();
  uint16_t dummy;
  lastStatus = readModbusRegister(MB_STATUS_REG, &dummy);
  if (!lastStatus) node->begin(address, Serial2);
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
  return node->writeSingleRegister(MB_SPEED_REG, speed);
}

bool SchneiderATV12::setMotorDirection(uint16_t direction) {
  uint16_t currentCmd;
  if (!node->readHoldingRegisters(MB_CMD_REG, 1)) return false;
  currentCmd = node->getResponseBuffer(0);
  if (direction) currentCmd |= (1 << 11);
  else currentCmd &= ~(1 << 11);
  return node->writeSingleRegister(MB_CMD_REG, currentCmd);
}

bool SchneiderATV12::sendCommandWithRetry(uint16_t command) {
  for (uint8_t i = 0; i < 3; i++) {
    if (node->writeSingleRegister(MB_CMD_REG, command)) return true;
    delay(100);
  }
  return false;
}

void SchneiderATV12::readFrequencyLimits() {
  node->readHoldingRegisters(MB_MAX_FREQ_REG, 1);
  maxFrequency = node->getResponseBuffer(0);
  node->readHoldingRegisters(MB_MIN_FREQ_REG, 1);
  minFrequency = node->getResponseBuffer(0);
  node->readHoldingRegisters(MB_HIGH_SPEED_REG, 1);
  highSpeed = node->getResponseBuffer(0);
}

void SchneiderATV12::readMotorStatus() {
  node->readHoldingRegisters(MB_STATUS_REG, 1);
  motorStatus = node->getResponseBuffer(0);
  node->readHoldingRegisters(MB_ACTUAL_SPEED_REG, 1);
  motorSpeed = node->getResponseBuffer(0);
  node->readHoldingRegisters(MB_ERRORS_REG, 1);
  motorErrors = node->getResponseBuffer(0);
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
  return node->writeSingleRegister(MB_SPEED_REG, speed);
}

bool SiemensV20::setMotorDirection(uint16_t direction) {
  uint16_t currentCmd;
  if (!node->readHoldingRegisters(MB_CMD_REG, 1)) return false;
  currentCmd = node->getResponseBuffer(0);
  if (direction) currentCmd |= (1 << 11);
  else currentCmd &= ~(1 << 11);
  return node->writeSingleRegister(MB_CMD_REG, currentCmd);
}

bool SiemensV20::sendCommandWithRetry(uint16_t command) {
  for (uint8_t i = 0; i < 3; i++) {
    if (node->writeSingleRegister(MB_CMD_REG, command)) return true;
    delay(100);
  }
  return false;
}

void SiemensV20::readFrequencyLimits() {
  node->readHoldingRegisters(MB_MAX_FREQ_REG, 1);
  maxFrequency = node->getResponseBuffer(0);
  node->readHoldingRegisters(MB_MIN_FREQ_REG, 1);
  minFrequency = node->getResponseBuffer(0);
}

void SiemensV20::readMotorStatus() {
  node->readHoldingRegisters(MB_STATUS_REG, 1);
  motorStatus = node->getResponseBuffer(0);
  node->readHoldingRegisters(MB_ACTUAL_SPEED_REG, 1);
  motorSpeed = node->getResponseBuffer(0);
  node->readHoldingRegisters(MB_ERRORS_REG, 1);
  motorErrors = node->getResponseBuffer(0);
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
  return node->writeSingleRegister(MB_SPEED_REG, speed);
}

bool WEGCFW500::setMotorDirection(uint16_t direction) {
  uint16_t currentCmd;
  if (!node->readHoldingRegisters(MB_CMD_REG, 1)) return false;
  currentCmd = node->getResponseBuffer(0);
  if (direction) currentCmd |= (1 << 11);
  else currentCmd &= ~(1 << 11);
  return node->writeSingleRegister(MB_CMD_REG, currentCmd);
}

bool WEGCFW500::sendCommandWithRetry(uint16_t command) {
  for (uint8_t i = 0; i < 3; i++) {
    if (node->writeSingleRegister(MB_CMD_REG, command)) return true;
    delay(100);
  }
  return false;
}

void WEGCFW500::readFrequencyLimits() {
  node->readHoldingRegisters(MB_MAX_FREQ_REG, 1);
  maxFrequency = node->getResponseBuffer(0);
  node->readHoldingRegisters(MB_MIN_FREQ_REG, 1);
  minFrequency = node->getResponseBuffer(0);
}

void WEGCFW500::readMotorStatus() {
  node->readHoldingRegisters(MB_STATUS_REG, 1);
  motorStatus = node->getResponseBuffer(0);
  node->readHoldingRegisters(MB_ACTUAL_SPEED_REG, 1);
  motorSpeed = node->getResponseBuffer(0);
  node->readHoldingRegisters(MB_ERRORS_REG, 1);
  motorErrors = node->getResponseBuffer(0);
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
  return node->writeSingleRegister(MB_SPEED_REG, speed);
}

bool ABBACS550::setMotorDirection(uint16_t direction) {
  uint16_t currentCmd;
  if (!node->readHoldingRegisters(MB_CMD_REG, 1)) return false;
  currentCmd = node->getResponseBuffer(0);
  if (direction) currentCmd |= (1 << 11);
  else currentCmd &= ~(1 << 11);
  return node->writeSingleRegister(MB_CMD_REG, currentCmd);
}

bool ABBACS550::sendCommandWithRetry(uint16_t command) {
  for (uint8_t i = 0; i < 3; i++) {
    if (node->writeSingleRegister(MB_CMD_REG, command)) return true;
    delay(100);
  }
  return false;
}

void ABBACS550::readFrequencyLimits() {
  node->readHoldingRegisters(MB_MAX_FREQ_REG, 1);
  maxFrequency = node->getResponseBuffer(0);
  node->readHoldingRegisters(MB_MIN_FREQ_REG, 1);
  minFrequency = node->getResponseBuffer(0);
}

void ABBACS550::readMotorStatus() {
  node->readHoldingRegisters(MB_STATUS_REG, 1);
  motorStatus = node->getResponseBuffer(0);
  node->readHoldingRegisters(MB_ACTUAL_SPEED_REG, 1);
  motorSpeed = node->getResponseBuffer(0);
  node->readHoldingRegisters(MB_ERRORS_REG, 1);
  motorErrors = node->getResponseBuffer(0);
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
  return node->writeSingleRegister(MB_SPEED_REG, speed);
}

bool DanfossFC302::setMotorDirection(uint16_t direction) {
  uint16_t currentCmd;
  if (!node->readHoldingRegisters(MB_CMD_REG, 1)) return false;
  currentCmd = node->getResponseBuffer(0);
  if (direction) currentCmd |= (1 << 11);
  else currentCmd &= ~(1 << 11);
  return node->writeSingleRegister(MB_CMD_REG, currentCmd);
}

bool DanfossFC302::sendCommandWithRetry(uint16_t command) {
  for (uint8_t i = 0; i < 3; i++) {
    if (node->writeSingleRegister(MB_CMD_REG, command)) return true;
    delay(100);
  }
  return false;
}

void DanfossFC302::readFrequencyLimits() {
  node->readHoldingRegisters(MB_MAX_FREQ_REG, 1);
  maxFrequency = node->getResponseBuffer(0);
  node->readHoldingRegisters(MB_MIN_FREQ_REG, 1);
  minFrequency = node->getResponseBuffer(0);
}

void DanfossFC302::readMotorStatus() {
  node->readHoldingRegisters(MB_STATUS_REG, 1);
  motorStatus = node->getResponseBuffer(0);
  node->readHoldingRegisters(MB_ACTUAL_SPEED_REG, 1);
  motorSpeed = node->getResponseBuffer(0);
  node->readHoldingRegisters(MB_ERRORS_REG, 1);
  motorErrors = node->getResponseBuffer(0);
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
  return node->writeSingleRegister(MB_SPEED_REG, speed);
}

bool YaskawaV1000::setMotorDirection(uint16_t direction) {
  uint16_t currentCmd;
  if (!node->readHoldingRegisters(MB_CMD_REG, 1)) return false;
  currentCmd = node->getResponseBuffer(0);
  if (direction) currentCmd |= (1 << 11);
  else currentCmd &= ~(1 << 11);
  return node->writeSingleRegister(MB_CMD_REG, currentCmd);
}

bool YaskawaV1000::sendCommandWithRetry(uint16_t command) {
  for (uint8_t i = 0; i < 3; i++) {
    if (node->writeSingleRegister(MB_CMD_REG, command)) return true;
    delay(100);
  }
  return false;
}

void YaskawaV1000::readFrequencyLimits() {
  node->readHoldingRegisters(MB_MAX_FREQ_REG, 1);
  maxFrequency = node->getResponseBuffer(0);
  node->readHoldingRegisters(MB_MIN_FREQ_REG, 1);
  minFrequency = node->getResponseBuffer(0);
}

void YaskawaV1000::readMotorStatus() {
  node->readHoldingRegisters(MB_STATUS_REG, 1);
  motorStatus = node->getResponseBuffer(0);
  node->readHoldingRegisters(MB_ACTUAL_SPEED_REG, 1);
  motorSpeed = node->getResponseBuffer(0);
  node->readHoldingRegisters(MB_ERRORS_REG, 1);
  motorErrors = node->getResponseBuffer(0);
  motorDirection = (motorStatus & 0x8000) ? 1 : 0;
}

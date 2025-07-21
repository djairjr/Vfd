#ifndef VFD_H
#define VFD_H

#include <ModbusMaster.h>

#define STATE_NOT_READY         0x0000
#define STATE_SWITCH_DISABLED   0x0040
#define STATE_READY             0x0021
#define STATE_SWITCHED_ON       0x0033
#define STATE_OPERATION_ENABLED 0x0037
#define STATE_FAULT             0x0008

#define CMD_SHUTDOWN         0x0006
#define CMD_SWITCH_ON        0x0007
#define CMD_RESET_FAULT      0x0080
#define CMD_DISABLE_OPERATION  0x0007
#define CMD_RUN_FORWARD      0x000F
#define CMD_RUN_REVERSE      0x080F

class VFD {
public:
  VFD(ModbusMaster* node, uint8_t address = 1);
  virtual bool setMotorSpeed(uint16_t speed) = 0;
  virtual bool setMotorDirection(uint16_t direction) = 0;
  virtual bool sendCommandWithRetry(uint16_t command) = 0;
  virtual void readFrequencyLimits() = 0;
  virtual void readMotorStatus() = 0;

  uint16_t getMotorSpeed() const { return motorSpeed; }
  uint16_t getMotorStatus() const { return motorStatus; }
  uint16_t getMotorErrors() const { return motorErrors; }
  uint16_t getMotorDirection() const { return motorDirection; }
  uint16_t getMinFrequency() const { return minFrequency; }
  uint16_t getMaxFrequency() const { return maxFrequency; }
  uint16_t getHighSpeed() const { return highSpeed; }

protected:
  ModbusMaster* node;
  uint8_t address;

  uint16_t MB_CMD_REG;
  uint16_t MB_SPEED_REG;
  uint16_t MB_ACTUAL_SPEED_REG;
  uint16_t MB_STATUS_REG;
  uint16_t MB_ERRORS_REG;
  uint16_t MB_MAX_FREQ_REG;
  uint16_t MB_MIN_FREQ_REG;
  uint16_t MB_HIGH_SPEED_REG;

  uint16_t motorSpeed = 0;
  uint16_t motorDirection = 0;
  uint16_t motorStatus = 0;
  uint16_t motorErrors = 0;
  uint16_t minFrequency = 0;
  uint16_t maxFrequency = 600;
  uint16_t highSpeed = 600;
};

class SchneiderATV12 : public VFD {
public:
  SchneiderATV12(ModbusMaster* node, uint8_t address = 1);
  bool setMotorSpeed(uint16_t speed) override;
  bool setMotorDirection(uint16_t direction) override;
  bool sendCommandWithRetry(uint16_t command) override;
  void readFrequencyLimits() override;
  void readMotorStatus() override;
};

class SiemensV20 : public VFD {
public:
  SiemensV20(ModbusMaster* node, uint8_t address = 1);
  bool setMotorSpeed(uint16_t speed) override;
  bool setMotorDirection(uint16_t direction) override;
  bool sendCommandWithRetry(uint16_t command) override;
  void readFrequencyLimits() override;
  void readMotorStatus() override;
};

class WEGCFW500 : public VFD {
public:
  WEGCFW500(ModbusMaster* node, uint8_t address = 1);
  bool setMotorSpeed(uint16_t speed) override;
  bool setMotorDirection(uint16_t direction) override;
  bool sendCommandWithRetry(uint16_t command) override;
  void readFrequencyLimits() override;
  void readMotorStatus() override;
};

class ABBACS550 : public VFD {
public:
  ABBACS550(ModbusMaster* node, uint8_t address = 1);
  bool setMotorSpeed(uint16_t speed) override;
  bool setMotorDirection(uint16_t direction) override;
  bool sendCommandWithRetry(uint16_t command) override;
  void readFrequencyLimits() override;
  void readMotorStatus() override;
};

class DanfossFC302 : public VFD {
public:
  DanfossFC302(ModbusMaster* node, uint8_t address = 1);
  bool setMotorSpeed(uint16_t speed) override;
  bool setMotorDirection(uint16_t direction) override;
  bool sendCommandWithRetry(uint16_t command) override;
  void readFrequencyLimits() override;
  void readMotorStatus() override;
};

class YaskawaV1000 : public VFD {
public:
  YaskawaV1000(ModbusMaster* node, uint8_t address = 1);
  bool setMotorSpeed(uint16_t speed) override;
  bool setMotorDirection(uint16_t direction) override;
  bool sendCommandWithRetry(uint16_t command) override;
  void readFrequencyLimits() override;
  void readMotorStatus() override;
};

#endif

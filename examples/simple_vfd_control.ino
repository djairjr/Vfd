#include <vfd.h>
#include <ModbusMaster.h>

// Initialize ModbusMaster instance
ModbusMaster node;

// Create VFD instance (replace with your model)
SchneiderATV12 vfd(&node, 1); // Address = 1

void setup() {
  Serial.begin(115200); // USB Debug
  Serial2.begin(19200); // Modbus communication
  
  vfd.readFrequencyLimits(); // Read min/max frequencies
  vfd.readMotorStatus();     // Read current status
}

void loop() {
  // Set motor speed to 50% of max frequency
  vfd.setMotorSpeed(vfd.getMaxFrequency() / 2);
  
  // Set forward direction
  vfd.setMotorDirection(0);
  
  // Start motor
  vfd.sendCommandWithRetry(CMD_RUN_FORWARD);
  
  delay(5000);
  
  // Read and display status
  vfd.readMotorStatus();
  Serial.print("Speed: "); Serial.println(vfd.getMotorSpeed());
  Serial.print("Status: "); Serial.println(vfd.getMotorStatus(), HEX);
}

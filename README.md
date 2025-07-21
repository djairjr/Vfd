# VFD Control Library
An Arduino library for controlling various Variable Frequency Drive (VFD) models via Modbus RTU communication. This library provides a unified interface to work with multiple VFD brands including Schneider, Siemens, WEG, ABB, Danfoss, and Yaskawa.

## Features

- Supports 6 major VFD brands/models
- Unified API for different manufacturers
- Modbus RTU communication
- Speed and direction control
- Status monitoring
- Error reporting
- Frequency limits management

## Supported VFD Models

- Schneider ATV12
- Siemens V20
- WEG CFW500
- ABB ACS550
- Danfoss FC302
- Yaskawa V1000

## Installation

1. **Arduino IDE**:
   - Download the latest release as a ZIP file
   - In Arduino IDE: Sketch > Include Library > Add .ZIP Library...
   - Select the downloaded file

2. **PlatformIO**:
   ```ini
   lib_deps = 
       https://github.com/yourusername/vfd-control.git
   ```

## Dependencies

- [ModbusMaster](https://github.com/4-20ma/ModbusMaster) library

## Hardware Setup

1. Connect your Arduino to the VFD via RS485:
   - Arduino pin D2 (or other) -> VFD RS485+
   - Arduino pin D3 (or other) -> VFD RS485-
   - Common ground

2. Configure VFD communication parameters:
   - Baud rate: 9600/19200/38400 (must match your code)
   - Parity: Usually none
   - Stop bits: Usually 1
   - Slave address: Set according to your VFD configuration

## Basic Usage

```cpp
#include <vfd.h>
#include <ModbusMaster.h>

// Initialize ModbusMaster instance
ModbusMaster node;

// Create VFD instance (replace with your model)
SchneiderATV12 vfd(&node, 1); // Address = 1

void setup() {
  Serial.begin(9600);
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
```

## API Reference

### Common Methods (Available for all VFD models)

#### Motor Control
- `bool setMotorSpeed(uint16_t speed)` - Set target speed (in Hz or RPM depending on VFD configuration)
- `bool setMotorDirection(uint16_t direction)` - Set motor direction (0 = forward, 1 = reverse)
- `bool sendCommandWithRetry(uint16_t command)` - Send control command with 3 retries

#### Status Monitoring
- `void readMotorStatus()` - Read current motor status (speed, direction, errors)
- `void readFrequencyLimits()` - Read min/max frequency limits

#### Getters
- `uint16_t getMotorSpeed()` - Get current motor speed
- `uint16_t getMotorStatus()` - Get current status word
- `uint16_t getMotorErrors()` - Get error codes
- `uint16_t getMotorDirection()` - Get current direction
- `uint16_t getMinFrequency()` - Get minimum frequency limit
- `uint16_t getMaxFrequency()` - Get maximum frequency limit
- `uint16_t getHighSpeed()` - Get high speed setting (Schneider specific)

### Command Constants
```cpp
#define STATE_NOT_READY         0x0000
#define STATE_SWITCH_DISABLED   0x0040
#define STATE_READY             0x0021
#define STATE_SWITCHED_ON       0x0033
#define STATE_OPERATION_ENABLED 0x0037
#define STATE_FAULT             0x0008

#define CMD_SHUTDOWN         0x0006
#define CMD_SWITCH_ON        0x0007
#define CMD_RESET_FAULT      0x0080
#define CMD_DISABLE_OPERATION 0x0007
#define CMD_RUN_FORWARD      0x000F
#define CMD_RUN_REVERSE      0x080F
```

## Model-Specific Notes

1. **Schneider ATV12**:
   - Additional `MB_HIGH_SPEED_REG` support
   - Speed values typically in 0.1 Hz units

2. **Siemens V20**:
   - Uses hexadecimal register addresses
   - May require specific parameter settings in VFD

3. **WEG CFW500**:
   - Speed reference in RPM by default

4. **ABB ACS550**:
   - Supports extended status monitoring

5. **Danfoss FC302**:
   - May require enable signal before commands

6. **Yaskawa V1000**:
   - Uses different register mapping scheme

## Troubleshooting

1. **No response from VFD**:
   - Check wiring (A+/B- or +/- connections)
   - Verify baud rate and address settings
   - Ensure VFD is in remote control mode

2. **Communication errors**:
   - Add termination resistors (120Ω) if using long cables
   - Check for ground loops
   - Try lower baud rates for long distances

3. **Command not working**:
   - Verify VFD is in the correct state (use state constants)
   - Check for active errors (readMotorStatus())

## Examples

The library includes several examples:
1. **BasicControl** - Simple speed and direction control
2. **StatusMonitor** - Display current motor status
3. **FrequencySweep** - Automatic frequency sweep within limits
4. **MultiVFD** - Control multiple VFDs of different brands

## License

MIT License - Free for personal and commercial use

## Contributing

Contributions are welcome! Please open issues or pull requests on GitHub.

## Support

For support, please open an issue on GitHub or contact [djair.jr@gmail.com].

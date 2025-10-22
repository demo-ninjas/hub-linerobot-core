# Hub LineRobot Core

**A comprehensive library for building intelligent line-following robots using the ESP32 based Hub LineRobot Breakout board.**

## Overview

The Hub LineRobot Core library provides a complete software foundation for building sophisticated line-following robots. Built specifically for tge Hub LineRobot Breakout board (based on an ESP32 microcontroller), it offers high-level abstractions for motor control, sensor management, LED control, WiFi connectivity, and web-based remote control.

### Key Features

- **🎯 Advanced Line Following**: Multi-sensor IR array with baseline calibration and PID control
- **📡 WiFi Connectivity**: Built-in web server with RESTful API for remote control and monitoring
- **🖥️ OLED Display**: Support for small-form LED displays
- **⚡ High Performance**: Takes advantage of both cores in on-board the ESP32S3 
- **📊 Comprehensive Diagnostics**: Built-in health monitoring and performance metrics

## The Hub LineRobot Breakout Board

The Hub LineRobot uses a custom designed development board featuring:

### Hardware Specifications
- **Microcontroller**: ESP32-S3 (dual-core 240MHz, WiFi enabled)
- **Power Supply**: 9-20V DC input with multiple voltage regulators offering access to 3.3v, 5v and 6v power
- **Motor Control**: Dual H-bridge motor driver (L293D compatible)
- **Sensors**: 4x downward-facing IR sensors
- **Accelerometer**: LIS3DH 3-axis accelerometer for orientation detection
- **LEDs**: 16 programmable LEDs via shift registers
- **Connectivity**: USB programming, WiFi, I2C headers and some GPIO headers for expansion

## Quick Start

### 1. Basic Line Following

```cpp
#include "linerobot_board.h"
#include "linerobot_state.h"

LineRobotState robot_state;
LineRobotBoard robot(&robot_state, Serial);

void setup() {
    if (!robot.begin()) {
        Serial.println("Robot initialization failed!");
        return;
    }
    Serial.println("Robot ready!");
}

void loop() {
    if (robot_state.currentState() == LINE_ROBOT_RACING) {
        // Simple proportional line following
        int left_sensor = robot.getIRSensorValue(2);
        int right_sensor = robot.getIRSensorValue(1);
        int error = right_sensor - left_sensor;
        
        int base_speed = 200;
        int correction = error * 0.8;
        
        robot.setMotorSpeed(base_speed - correction, base_speed + correction);
    } else {
        robot.setMotorSpeed(0, 0);
    }
}
```

### 2. WiFi-Enabled Remote Control

```cpp
LineRobotBoard robot(&robot_state, Serial, "WiFiSSID", "WifiPassword");

void setup() {
    robot.begin();
    
    // Robot automatically starts web server
    // Access via browser at robot's IP address
    // If you attach a micro OLED display, by default the IP address of the robot will be displayed on the screen 
    // It will also be printed to the Serial if debug is enabled (last parameter in the LineRobotBoard constructor, or via robot.setDebug(true))  
    Serial.println("Robot ready with WiFi control!");
}

void loop() {}
```

## Documentation

Comprehensive documentation is available in the [`docs/`](docs/) directory:

- **[📚 API Reference](docs/api-reference.md)** - Complete class documentation and method references
- **[🔧 Hardware Guide](docs/hardware-guide.md)** - Board layout, wiring, and assembly instructions  
- **[💡 Examples](docs/examples.md)** - Bunch of code examples to get you started

### Built-in Web Interface

When WiFi is enabled, the robot provides a web interface accessible via browser:

| Endpoint | Description |
|----------|-------------|
| `/` | Homepage |
| `/health` | For a quick "are you up?" check |
| `/status` | Quick JSON status of robot state |
| `/diagnostic` | Comprehensive diagnostic report |
| `/debug` | Summary of memory state |
| `/ir` | IR Sensor Threshold state |
| `/set-led` | Turn an on-board LED on/off |
| `/set-motor` | Manually set Motor speeds |
| `/pose` | Current 3-axis pose approximation of the robot |
| `/adc` | Get raw ADC values from the ESP |
| `/baseline` | Sensor calibration |

## Installation

### For PlatformIO Projects

Add to your `platformio.ini`:

```ini
[env:esp32]
platform = espressif32
board = hub-robot-v2
framework = arduino

lib_deps = 
    https://github.com/demo-ninjas/hub-linerobot-core.git
```

### For Arduino IDE

1. Download this repository as ZIP
2. Install via **Sketch → Include Library → Add .ZIP Library**
3. Install ESP32 board support via Board Manager

## Robot States & Behavior

By default, the robot operates using a basic state machine:

- **IDLE** (0): Robot Idle, ready for commands
- **PRIMED** (1): Ready to start line following
- **RACING** (2): Actively following line
- **BASELINE_INIT** (20): Preparing for sensor calibration
- **BASELINING** (21): Performing sensor calibration

**Button Controls:**
- Single press: Advance through states (IDLE → PRIMED → RACING)
- Long press: Enter calibration mode
- Double press: Reset baselines

## Development Setup

### Prerequisites

- **PlatformIO Tools**: [https://platformio.org/](https://platformio.org/)
- **VS Code**: [https://code.visualstudio.com/](https://code.visualstudio.com/)
- **C/C++ compiler** and standard development tools

### Required VS Code Extensions

- [C/C++ Extension Pack](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools-extension-pack)
- [PlatformIO IDE](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide)

### Building & Uploading

```bash
# Build the project
pio run

# Upload to robot
pio run --target upload

# Monitor serial output
pio device monitor
```

## Contributing

This library is designed for educational robotics purposes, it's not designed to be robust enough for commercial use.

Contributions are welcome!

### Publishing Changes

1. Update version in [`library.json`](library.json)
2. Test thoroughly with hardware
3. Update documentation as needed
4. Create pull request

**Note**: This library is currently distributed via GitHub. Include the repository URL in your `platformio.ini` dependencies.

## License

See [LICENSE](LICENSE) file for details.

## Support

- **Issues**: Report bugs and feature requests via GitHub Issues
- **Documentation**: Complete guides available in [`docs/`](docs/)
- **Examples**: Sample code in [`docs/examples.md`](docs/examples.md)


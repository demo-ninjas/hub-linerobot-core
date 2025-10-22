# Hardware Setup Guide

## Table of Contents

- [Hub Line Robot Board Layout](#hub-line-robot-board-layout)
- [Power Requirements](#power-requirements)
- [IR Sensor Assembly](#ir-sensor-assembly)
- [Motor Connections](#motor-connections)
- [LED System](#led-system)
- [OLED Display](#oled-display)
- [Accelerometer](#accelerometer)
- [Button Interface](#button-interface)
- [WiFi Antenna](#wifi-antenna)
- [Assembly Steps](#assembly-steps)
- [Testing & Calibration](#testing--calibration)
- [Troubleshooting](#troubleshooting)

---

## Hub Line Robot Board Layout

The Hub LineRobot uses a custom development board as its main controller. 
This board provides:
- ESP32S3 - with the following: 
  - Dual-core processing (240MHz)
  - WiFi connectivity
  - 16 ADC channels
  - 45 GPIO pins
  - USB connectivity
- Power via USB or Battery (min 9v, recommend >=11v)
- H-bridge Motor controller
- On-board Accelerometer
- 2 x daisy-chained Shift Registers
- 4 x downward-facing IR Sensors (IR Transmitter + Receiver)
- Numerous program controllable on-board LEDs
- Numerous I2C, GPIO + power headers for customisation


### Pin Assignments

| Function | Pin(s) | Notes |
|----------|--------|--------|
| IR Sensors | GPIO4, GPIO5, GPIO6, GPIO7 | ADC1_CH0-3 |
| Motor PWM | GPIO12, GPIO38 | Left/Right motor EN control |
| Motor Direction | GPIO13/GPIO14 + GPIO39/GPIO40 | Forward/Reverse control |
| Shift Register | GPIO11, GPIO9, GPIO10 | Data, Clock, Latch |
| I2C Bus | GPIO15 (SDA), GPIO16 (SCL) | OLED, Accelerometer, other peripherals |
| Button Input | GPIO21 | Pull-up enabled |
| Status LED | GPIO2 | Built-in indicator |

---

## Power Requirements

### Main Power Supply
- **Input Voltage:** 9-20v DC (recommended: 11-15v)
- **Current Draw:** 2-4A peak (depends on motor load)

### Power Distribution
```
Main Supply → 3.3v Voltage Regulator (ESP, accelerometer) 
            → 5v Voltage Regulator (IR LEDs, Shift Registers, Motor Controller)
            → 6v Voltage Regulator (Motors)
```

### Battery Operation
For mobile operation, use:
- **6x AA batteries** (9V nominal)
- **3S LiPo battery** (11.1v nominal)
- **4S LiPo battery** (14.8v nominal)
- **9V alkaline battery** (limited runtime, recommend 2 x 9v connected in parallel)

---

## IR Sensor Assembly

### Sensor Placement
The robot uses 4 infrared sensors arranged in this configuration:

```
Front of Robot
    [3] [2] [1] [0]
   Left        Right
```

- **Sensor 0:** Outer Right (GPIO4)
- **Sensor 1:** Inner Right (GPIO5)  
- **Sensor 2:** Inner Left (GPIO6)
- **Sensor 3:** Outer Left (GPIO7)


## Motor Connections

### Motor Driver Circuit
The robot uses a dual Full-bridge motor driver (L293D):

```
ESP32 GPIO12 → EN A (Left Motor Speed)
ESP32 GPIO13 → IN1 (Left Motor Direction control)
ESP32 GPIO14 → IN2 (Left Motor Direction control)
ESP32 GPIO38 → EN B (Right Motor Speed)
ESP32 GPIO38 → IN1 (Right Motor Direction control)
ESP32 GPIO40 → IN2 (Right Motor Direction control)
```

### Motor Specifications
- **Type:** DC geared motors
- **Voltage:** 6V nominal (5-9V range)
- **Current:** 200-800mA per motor
- **Speed:** 100-300 RPM
- **Torque:** 1-3 kg⋅cm

### Wheel Configuration
- **Diameter:** 65-70mm recommended
- **Width:** 15-20mm
- **Material:** Rubber for traction
- **Mounting:** Secure to motor shaft

---

## LED System

### Shift Register Configuration
The robot uses 2 x 74HC595 shift registers to control 16 LEDs:

```
ESP32 GPIO11 → Data (DS)
ESP32 GPIO9  → Clock (SHCP)
ESP32 GPIO10 → Latch (STCP)
5V → VCC, OE tied to ground
```

### LED Array Layout
```
IR Indicators:      [4][7][ 6][ 3]
IR LEDs:            [11][10][ 9][ 8]
Traffic Light LEDs: [0][1][2]
Board Btn LED:      [5]
Unboard Header LEDs:[15][14][13][12]
```

---

## OLED Display

### I2C Connection
```
I2C Header SDA → OLED SDA
I2C Header SCL → OLED SCL
I2C Header 3.3V → OLED VCC
I2C Header GND → OLED GND
```

### Supported Displays
- **SSD1306:** 128x64, 0.96" (most common)
- **SH1106:** 128x64, 1.3" (alternative)
- **Address:** 0x3C (default) or 0x3D

---

## Accelerometer

The board has a LIS3DH accelerometer attached.

### Settings
- **I2C Address:** 0x18 (SDO low) or 0x19 (SDO high)
- **Sample Rate:** 100Hz
- **Range:** ±4g (sufficient for robot motion)
- **Resolution:** 12-bit

### Mounting Orientation
- **X-axis:** Fordward-back
- **Y-axis:** Left-Right
- **Z-axis:** Up-down

### Calibration
The accelerometer is used for:
- Detecting robot orientation (eg. upside down or on the side)
- Motion detection
- Tilt compensation

---

## Button Interface

### Primary Button
```
ESP32 GPIO21 → Button (one side)
Button (other side) → Ground
Internal pull-up enabled
```

### Additional Button
An additional alternative button pin can be configured, to enable attaching an external button to the board.

```cpp
robot.addAlternateBoardButtonPin(pin_number, active_low);
```

### Button Functions
- **Single press:** State transitions
- **Double press:** Special functions
- **Long press:** Enter baseline mode

---

## WiFi Antenna

### Built-in Antenna
The ESP32-S3 has a built-in PCB antenna:
- **Range:** 50-100m open air
- **Frequency:** 2.4GHz (802.11 b/g/n)
- **Keep clear:** 5mm clearance around antenna area


---

## Troubleshooting

### Power Issues

**Symptom:** Robot won't turn on
- Check power supply voltage (9-20V)
- Test voltage regulator outputs using a multimeter

**Symptom:** Robot resets randomly
- Could be insufficient current capacity (check battery capacity)
- Voltage drops during motor operation (are you changing motor speeds significantly too quickly)
- Possibly a stack overflow - check how deep your going in the stack (if you're robot is connected to wifi, call: /debug to see memory info)
- Are you trying to access NVM? Make sure to pause the timer tasks before doing that
- Run out of heap memory perhaps? Check you haven't gone too crazy with memory usage  (if you're robot is connected to wifi, call: /debug to see memory info)

### Communication Problems

**Symptom:** WiFi won't connect
- Verify SSID and password
- Check antenna placement
- Test with simple WiFi sketch
- Ensure 2.4GHz network is available

### Sensor Issues

**Symptom:** IR sensors always triggered
- Check ambient lighting (particularly high levels of IR light from the sun for example)
- Check if you need to adjust the sensitivity using the on-board potentiometer
- Verify sensor height (3-8mm)
- Recalibrate baselines
- Check for electrical noise

### Motor Problems

**Symptom:** Motors don't respond or lose power quickly after starting
- Verify power supply voltage is 9v or above
- Verify power supply current capacity is sufficient
- Test with direct GPIO control
- Check for driver overheating

---

## Safety Guidelines

### Electrical Safety
- Always disconnect power when making connections
- Ensure proper (and common) grounding of all circuits
- Test with multimeter before applying power


---

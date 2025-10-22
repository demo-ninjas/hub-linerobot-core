# API Reference - Hub LineRobot Core

## Table of Contents

- [LineRobotBoard Class](#linerobotboard-class)
- [Constructor & Initialization](#constructor--initialization)
- [Motor Control](#motor-control)
- [Sensor Management](#sensor-management)
- [LED Control](#led-control)
- [Display Control](#display-control)
- [Network & HTTP](#network--http)
- [Configuration & Storage](#configuration--storage)
- [Diagnostics & Status](#diagnostics--status)
- [Button Handling](#button-handling)
- [LineRobotState Class](#linerobotstate-class)
- [InfraredSensor Class](#infraredsensor-class)

---

## LineRobotBoard Class

The main class providing complete robot functionality.

### Constructor & Initialization

#### `LineRobotBoard()`
```cpp
explicit LineRobotBoard(LineRobotState* state, 
                        CachingPrinter& logger,
                        const String& wifi_ssid = "", 
                        const String& wifi_password = "",
                        OledDisplayType oled_type = OLED_TYPE_NONE,
                        bool debug = false);
```

**Parameters:**
- `state` - Robot state object (required, must not be null)
- `logger` - Logging interface (required, typically Serial)
- `wifi_ssid` - WiFi network name (optional)
- `wifi_password` - WiFi password (optional)
- `oled_type` - OLED display type (optional)
- `debug` - Enable debug logging (optional)

**Throws:** `std::invalid_argument` if state or logger is null

#### `begin()`
```cpp
bool begin(uint16_t ir_threshold = DEFAULT_IR_THRESHOLD);
```
Initialize all hardware components and start internal tasks.

**Parameters:**
- `ir_threshold` - IR sensor trigger threshold (0-4095, default: 400)

**Returns:** `true` if initialization successful, `false` otherwise

#### `end()`
```cpp
void end();
```
Stop all tasks and cleanup resources. Call before destructor.

#### `isInitialized()`
```cpp
bool isInitialized() const;
```
**Returns:** `true` if board is successfully initialized

---

## Motor Control

### Basic Motor Control

#### `setMotorSpeedLeft()`
```cpp
bool setMotorSpeedLeft(int16_t speed, uint16_t ramp_time_ms = 0);
```

#### `setMotorSpeedRight()`
```cpp
bool setMotorSpeedRight(int16_t speed, uint16_t ramp_time_ms = 0);
```

#### `setMotorSpeedBoth()`
```cpp
bool setMotorSpeedBoth(int16_t speed, uint16_t ramp_time_ms = 0);
```

#### `setMotorSpeed()`
```cpp
bool setMotorSpeed(int16_t left, int16_t right, uint16_t ramp_time_ms = 0);
```

**Parameters:**
- `speed`, `left`, `right` - Motor speed (-255 to 255, negative = reverse)
- `ramp_time_ms` - Time to ramp to new speed (0 = immediate)

**Returns:** `true` if speed was set successfully

### Motor Status & Control

#### `getMotorSpeedLeft()`
```cpp
int16_t getMotorSpeedLeft() const;
```

#### `getMotorSpeedRight()`
```cpp
int16_t getMotorSpeedRight() const;
```

**Returns:** Current motor speed (-255 to 255)

#### `emergencyStop()`
```cpp
void emergencyStop();
```
Immediately stop both motors for safety.

---

## Sensor Management

### IR Sensor Reading

#### `getRawIRSensorValue()`
```cpp
uint16_t getRawIRSensorValue(uint8_t sensor) const;
```

#### `getIRSensorValue()`
```cpp
int16_t getIRSensorValue(uint8_t sensor) const;
```

**Parameters:**
- `sensor` - Sensor index (0-3): 0=outer right, 1=inner right, 2=inner left, 3=outer left

**Returns:** 
- Raw: ADC value (0-4095) or `INVALID_ANALOGUE_CHANNEL` if error
- Adjusted: Baseline-adjusted value or `INT16_MAX` if error

#### `isIRSensorTriggered()`
```cpp
bool isIRSensorTriggered(uint8_t sensor) const;
```

#### `getIRSensorMask()`
```cpp
uint8_t getIRSensorMask() const;
```

**Returns:** 4-bit mask where bit N represents sensor N trigger state

### IR Sensor Configuration

#### `setIRSensorThreshold()`
```cpp
bool setIRSensorThreshold(uint8_t sensor, uint16_t threshold);
```

#### `setAllIRSensorThresholds()`
```cpp
void setAllIRSensorThresholds(uint16_t threshold);
```

Set the trigger threshold for IR sensors.

### Baseline Calibration

#### `baselineIRSensors()`
```cpp
bool baselineIRSensors();
```
Automatically calibrate all IR sensor baselines. Robot must be on ground, OFF the track.

#### `setIRSensorBaseline()`
```cpp
bool setIRSensorBaseline(uint8_t sensor, int16_t baseline);
```
Manually set baseline for a specific sensor.

#### `resetIRBaselines()`
```cpp
bool resetIRBaselines();
```
Reset all baselines to zero and clear saved values.

### Other Sensors

#### `readAnalogueValue()`
```cpp
uint16_t readAnalogueValue(uint8_t pin) const;
```
Read raw ADC value from any GPIO pin.

#### `getAccelerometerReading()`
```cpp
bool getAccelerometerReading(float& x, float& y, float& z) const;
```
Get current accelerometer readings in g-force units.

---

## LED Control

#### `setLED()`
```cpp
bool setLED(uint8_t led, bool on);
```
Control individual LED (0-15).

#### `setLEDMask()`
```cpp
void setLEDMask(uint16_t mask);
```
Set multiple LEDs using bitmask (more efficient than individual calls).

#### `getLEDMask()`
```cpp
uint16_t getLEDMask() const;
```
Get current state of all LEDs as bitmask.

---

## Display Control

#### `setOLEDLine()`
```cpp
bool setOLEDLine(uint8_t line, const String& text, bool centered = true);
```

**Parameters:**
- `line` - Line number (0-5)
- `text` - Text to display
- `centered` - Center text on line

#### `clearOLED()`
```cpp
void clearOLED();
```

---

## Network & HTTP

#### `addHttpRoute()`
```cpp
bool addHttpRoute(const String& route, 
                  std::function<void(HttpRequest*, HttpResponse*, LineRobotBoard*)> handler);
```

Add custom HTTP API endpoint.

#### `isWiFiConnected()`
```cpp
bool isWiFiConnected() const;
```

### Built-in HTTP Endpoints

| Endpoint | Description | Parameters |
|----------|-------------|------------|
| `/` | Main page | - |
| `/status` | System status (JSON) | - |
| `/diagnostic` | Full diagnostic report | - |
| `/ir` | IR sensor data | `Accept: application/json` |
| `/pose` | Accelerometer data | `Accept: application/json` |
| `/adc` | ADC readings | `Accept: application/json` |
| `/set-led` | Control LEDs | `?0=true&1=false&all=true` |
| `/set-motor` | Control motors | `?left=200&right=150&both=100&stop=1` |
| `/baseline` | Calibration | `?action=start` or `?action=reset` |

---

## Configuration & Storage

#### `saveConfiguration()`
```cpp
bool saveConfiguration();
```

#### `setAutoBaseline()`
```cpp
void setAutoBaseline(bool enable);
```
Enable/disable automatic baseline correction (future feature).

---

## Diagnostics & Status

#### `performDiagnostic()`
```cpp
String performDiagnostic() const;
```
Generate comprehensive diagnostic report.

#### `getSystemStatus()`
```cpp
String getSystemStatus() const;
```
Get JSON-formatted system status.

#### `setDebug()`
```cpp
void setDebug(bool debug);
```

#### `isDebugEnabled()`
```cpp
bool isDebugEnabled() const;
```

#### `getUptime()`
```cpp
unsigned long getUptime() const;
```
Get system uptime in milliseconds.

#### `getCoreTasksTickCount()`
```cpp
unsigned long getCoreTasksTickCount() const;
```

Note: THe tick count is wrapped every 60k ticks (every 1 min)

#### `getOtherTasksTickCount()`
```cpp
unsigned long getOtherTasksTickCount() const;
```

Note: The tick count is wrapped every 3600 ticks (every 1 min)

---

## Button Handling

#### `addAlternateBoardButtonPin()`
```cpp
bool addAlternateBoardButtonPin(uint8_t pin, bool active_low = true);
```
Add additional button input pin.

#### `onBoardButtonPressed()`
```cpp
void onBoardButtonPressed(std::function<bool(long)> callback);
```

#### `onBoardButtonDoublePressed()`
```cpp
void onBoardButtonDoublePressed(std::function<bool(long)> callback);
```

#### `onBoardButtonLongPressed()`
```cpp
void onBoardButtonLongPressed(std::function<bool(long)> callback);
```

**Callback Parameters:**
- `long` - Time pressed (ms) or time between presses (double press)

**Callback Return:**
- `true` - Event handled, don't run default behavior
- `false` - Run default behavior

### Default Button Behavior

| Action | From State | Result |
|--------|------------|--------|
| Press | IDLE | → PRIMED |
| Press | PRIMED | → RACING |
| Press | RACING | → IDLE |
| Press | IDLE (vertical) | → BASELINE_INIT |
| Press | BASELINE_INIT | Start calibration |
| Double Press | BASELINE_INIT | Clear baselines → IDLE |
| Long Press | IDLE | → BASELINE_INIT |
| Long Press | PRIMED/BASELINE | → IDLE |

---

## LineRobotState Class

### Constructor
```cpp
LineRobotState();
```

### State Management
```cpp
uint8_t currentState();
uint8_t previousState();
void setState(uint8_t state);
```

### State Constants
```cpp
#define LINE_ROBOT_IDLE         0
#define LINE_ROBOT_PRIMED       1 
#define LINE_ROBOT_RACING       2
#define LINE_ROBOT_BASELINE_INIT 20
#define LINE_ROBOT_BASELINING   21
```

### State Callbacks
```cpp
void onIDLEStart(std::function<void()> callback);
void onIDLEEnd(std::function<void()> callback);
void onPRIMEDStart(std::function<void()> callback);
void onPRIMEDEnd(std::function<void()> callback);
void onRACINGStart(std::function<void()> callback);
void onRACINGEnd(std::function<void()> callback);
void onBASELINEINITStart(std::function<void()> callback);
void onBASELINEINITEnd(std::function<void()> callback);
void onBASELININGStart(std::function<void()> callback);
void onBASELININGEnd(std::function<void()> callback);
void onStateChange(std::function<void(uint8_t, uint8_t)> callback);
```

### Public Data Members
```cpp
LineRobotPose pose;           // Robot pose and acceleration data
int motor_speed_left;         // Left motor speed (-255 to 255)
int motor_speed_right;        // Right motor speed (-255 to 255)

// IR sensor values (baseline-adjusted)
int ir_outer_left;
int ir_inner_left; 
int ir_inner_right;
int ir_outer_right;

// IR sensor trigger states
bool ir_outer_left_triggered;
bool ir_inner_left_triggered;
bool ir_inner_right_triggered;  
bool ir_outer_right_triggered;
```

---

## InfraredSensor Class

### Constructor
```cpp
InfraredSensor(uint8_t analog_pin, uint8_t indicator_led_num, 
               uint8_t ir_led_num, uint16_t threshold = DEFAULT_IR_THRESHOLD,
               int16_t baseline = 0);
```

### Core Methods
```cpp
void tick(bool force_update = false);
void setThreshold(uint16_t threshold);
void setBaseline(int16_t baseline);
void reset();
```

### Getters
```cpp
int16_t getValue() const;
uint16_t getRawValue() const;
int16_t getBaseline() const;
uint16_t getThreshold() const;
uint8_t getAnalogPin() const;
uint8_t getIndicatorLedNum() const;
uint8_t getIrLedNum() const;
bool isTriggered() const;
bool hasStateChanged() const;
```

### Timing Information
```cpp
unsigned long timeInLastState() const;
unsigned long timeInCurrentState() const;
```

### Utility
```cpp
String describe() const;
String describeJson() const;
```

---

## Constants

```cpp
#define BOARD_VERSION "2.1"
#define BOARD_TYPE "ESP32"
#define NUM_IR_SENSORS 4
#define MAX_ANALOG_VALUE 4095
#define MIN_MOTOR_SPEED -255
#define MAX_MOTOR_SPEED 255
#define DEFAULT_IR_THRESHOLD 400
#define INVALID_ANALOGUE_CHANNEL 65535
```

---

## Error Handling

Most methods return `bool` to indicate success/failure. Check return values for error detection:

```cpp
if (!robot.begin()) {
    Serial.println("Robot initialization failed!");
    return;
}

if (!robot.setMotorSpeedLeft(200)) {
    Serial.println("Invalid motor speed!");
}
```

For methods returning values, check for special error constants:
- `INVALID_ANALOGUE_CHANNEL` for ADC readings
- `INT16_MAX` for invalid sensor readings

---

## Thread Safety

The following operations are thread-safe:
- All sensor readings
- Motor control operations  
- LED control
- Configuration access

Non-thread-safe operations:
- Initialization (`begin()`)
- Shutdown (`end()`)
- HTTP route registration

---

## Performance Notes

- **High-frequency operations** (IR sensors, accelerometer) run on Core1 at 2kHz
- **Low-frequency operations** (HTTP, buttons) run on Core0 at 60.5Hz
- **LED updates** are batched for efficiency
- **ADC readings** are cached to minimize overhead
- **Memory allocation** uses smart pointers for automatic cleanup
# Examples - LineRobot Programming

This directory contains example code demonstrating various LineRobot features and programming patterns.

## Table of Contents

- [Basic Examples](#basic-examples)
- [Advanced Examples](#advanced-examples)
- [Line Following Algorithms](#line-following-algorithms)
- [Custom HTTP APIs](#custom-http-apis)
- [State Machine Programming](#state-machine-programming)
- [Sensor Processing](#sensor-processing)

---

## Basic Examples

### 1. Simple Line Following (`simple_line_follow.ino`)

Basic line following with proportional control:

```cpp
#include "linerobot_board.h"
#include "linerobot_state.h"

LineRobotState robot_state;
LineRobotBoard robot(&robot_state, Serial);

void setup() {
    Serial.begin(115200);
    
    if (!robot.begin()) {
        Serial.println("Robot initialization failed!");
        return;
    }
    
    Serial.println("Robot ready! Press button to start.");
}

void loop() {
    // Simple proportional line following
    if (robot_state.currentState() == LINE_ROBOT_RACING) {
        // Read sensor values  
        int left_inner = robot.getIRSensorValue(2);
        int right_inner = robot.getIRSensorValue(1);
        
        // Calculate error (positive = line to right)
        int error = right_inner - left_inner;
        
        // Proportional control
        int base_speed = 150;
        float kp = 0.8;
        int correction = kp * error;
        
        // Apply motor speeds
        int left_speed = base_speed - correction;
        int right_speed = base_speed + correction;
        
        robot.setMotorSpeed(left_speed, right_speed);
    } else {
        // Stop motors when not racing
        robot.setMotorSpeed(0, 0);
    }
}
```

### 2. Motor Control Demo (`motor_demo.ino`)

Demonstrate various motor control patterns:

```cpp
#include "linerobot_board.h"
#include "linerobot_state.h"

LineRobotState robot_state;
LineRobotBoard robot(&robot_state, Serial);

void setup() {
    Serial.begin(115200);
    robot.begin();
    
    // Set up state callbacks
    robot_state.onRACINGStart([]() {
        Serial.println("Starting motor demo...");
        runMotorDemo();
    });
}

void runMotorDemo() {
    // Forward at different speeds
    Serial.println("Forward slow");
    robot.setMotorSpeedBoth(100);
    delay(2000);
    
    Serial.println("Forward fast");  
    robot.setMotorSpeedBoth(200);
    delay(2000);
    
    // Turning
    Serial.println("Turn right");
    robot.setMotorSpeed(150, 50);
    delay(1000);
    
    Serial.println("Turn left");
    robot.setMotorSpeed(50, 150); 
    delay(1000);
    
    // Spin in place
    Serial.println("Spin right");
    robot.setMotorSpeed(150, -150);
    delay(1000);
    
    Serial.println("Spin left");
    robot.setMotorSpeed(-150, 150);
    delay(1000);
    
    // Gradual speed changes
    Serial.println("Gradual acceleration");
    robot.setMotorSpeedBoth(0, 2000);  // Ramp over 2 seconds
    delay(3000);
    
    Serial.println("Emergency stop");
    robot.emergencyStop();
    
    // Return to idle
    robot_state.setState(LINE_ROBOT_IDLE);
}

void loop() {
    // Nothing needed in loop - callbacks handle everything
}
```

### 3. Sensor Reading Demo (`sensor_demo.ino`)

Read and display all sensor values:

```cpp
#include "linerobot_board.h"
#include "linerobot_state.h"

LineRobotState robot_state;
LineRobotBoard robot(&robot_state, Serial);

void setup() {
    Serial.begin(115200);
    robot.begin();
    robot.setDebug(true);
}

void loop() {
    // IR Sensors
    Serial.println("=== IR Sensors ===");
    for (int i = 0; i < 4; i++) {
        uint16_t raw = robot.getRawIRSensorValue(i);
        int16_t adjusted = robot.getIRSensorValue(i);
        bool triggered = robot.isIRSensorTriggered(i);
        
        Serial.printf("Sensor %d: Raw=%4d, Adj=%4d, Trig=%s\n",
                      i, raw, adjusted, triggered ? "YES" : "NO");
    }
    
    // Sensor mask (more efficient for multiple sensors)
    uint8_t mask = robot.getIRSensorMask();
    Serial.printf("Sensor mask: 0x%02X (binary: %04b)\n", mask, mask);
    
    // Accelerometer
    float x, y, z;
    if (robot.getAccelerometerReading(x, y, z)) {
        Serial.printf("Accelerometer: X=%.2f, Y=%.2f, Z=%.2f g\n", x, y, z);
    } else {
        Serial.println("Accelerometer: Read failed");
    }
    
    // Motor status
    Serial.printf("Motors: Left=%d, Right=%d\n", 
                  robot.getMotorSpeedLeft(), robot.getMotorSpeedRight());
    
    // System status
    Serial.printf("State: %d, Uptime: %lu ms\n", 
                  robot_state.currentState(), robot.getUptime());
    
    Serial.println();
    delay(500);
}
```

---

## Advanced Examples

### 4. PID Line Following (`pid_line_follow.ino`)

Advanced line following with PID control:

```cpp
#include "linerobot_board.h" 
#include "linerobot_state.h"

LineRobotState robot_state;
LineRobotBoard robot(&robot_state, Serial);

// PID Controller class
class PIDController {
private:
    float kp, ki, kd;
    float integral, last_error;
    unsigned long last_time;
    
public:
    PIDController(float p, float i, float d) : kp(p), ki(i), kd(d) {
        reset();
    }
    
    void reset() {
        integral = 0;
        last_error = 0;
        last_time = millis();
    }
    
    float calculate(float error) {
        unsigned long now = millis();
        float dt = (now - last_time) / 1000.0;
        last_time = now;
        
        // Avoid division by zero
        if (dt <= 0) dt = 0.001;
        
        // PID calculation
        integral += error * dt;
        float derivative = (error - last_error) / dt;
        
        float output = kp * error + ki * integral + kd * derivative;
        
        last_error = error;
        return output;
    }
    
    void setTunings(float p, float i, float d) {
        kp = p; ki = i; kd = d;
    }
};

PIDController pid(1.2, 0.1, 0.05);  // Tuned values

void setup() {
    Serial.begin(115200);
    robot.begin();
    
    robot_state.onRACINGStart([]() {
        Serial.println("Starting PID line following");
        pid.reset();
    });
    
    robot_state.onRACINGEnd([]() {
        robot.emergencyStop();
        Serial.println("PID following stopped");
    });
}

void loop() {
    if (robot_state.currentState() == LINE_ROBOT_RACING) {
        // Calculate line position using weighted average
        float position = calculateLinePosition();
        
        // PID control
        float error = position;  // Target position is 0 (center)
        float correction = pid.calculate(error);
        
        // Apply correction to motors
        int base_speed = 180;
        int left_speed = constrain(base_speed - correction, -255, 255);
        int right_speed = constrain(base_speed + correction, -255, 255);
        
        robot.setMotorSpeed(left_speed, right_speed);
        
        // Debug output
        static unsigned long last_debug = 0;
        if (millis() - last_debug > 100) {
            Serial.printf("Pos: %6.2f, Error: %6.2f, Correction: %6.2f, L: %3d, R: %3d\n",
                          position, error, correction, left_speed, right_speed);
            last_debug = millis();
        }
    }
}

float calculateLinePosition() {
    // Read all sensors
    int sensors[4];
    for (int i = 0; i < 4; i++) {
        sensors[i] = max(0, robot.getIRSensorValue(i));
    }
    
    // Calculate weighted position (-1.5 to +1.5)
    // Negative = line to left, Positive = line to right
    float numerator = 0;
    float denominator = 0;
    
    float positions[4] = {1.5, 0.5, -0.5, -1.5};  // Right to left
    
    for (int i = 0; i < 4; i++) {
        numerator += sensors[i] * positions[i];
        denominator += sensors[i];
    }
    
    if (denominator == 0) {
        // No line detected - use last known position or default
        return 0;  // Assume centered
    }
    
    return numerator / denominator;
}
```

### 5. WiFi Remote Control (`wifi_control.ino`)

Remote control via WiFi with custom HTTP endpoints:

```cpp
#include "linerobot_board.h"
#include "linerobot_state.h"

LineRobotState robot_state;
LineRobotBoard robot(&robot_state, Serial, "YourWiFi", "YourPassword");

void setup() {
    Serial.begin(115200);
    robot.begin();
    
    // Add custom HTTP routes
    setupCustomRoutes();
    
    Serial.println("Robot ready!");
    Serial.print("IP Address: ");
    // Will print IP when connected
}

void setupCustomRoutes() {
    // Manual drive control
    robot.addHttpRoute("/drive", [](HttpRequest* req, HttpResponse* res, LineRobotBoard* bot) {
        String direction = req->getParam("dir");
        String speed_str = req->getParam("speed");
        int speed = speed_str.toInt();
        
        if (speed < 0) speed = 0;
        if (speed > 255) speed = 255;
        
        if (direction == "forward") {
            bot->setMotorSpeedBoth(speed);
        } else if (direction == "backward") {
            bot->setMotorSpeedBoth(-speed);
        } else if (direction == "left") {
            bot->setMotorSpeed(-speed/2, speed);
        } else if (direction == "right") {
            bot->setMotorSpeed(speed, -speed/2);
        } else if (direction == "stop") {
            bot->setMotorSpeedBoth(0);
        }
        
        res->setStatusCode(200);
        res->setHeader("Content-Type", "application/json");
        res->printf("{\"status\":\"ok\",\"direction\":\"%s\",\"speed\":%d}", 
                    direction.c_str(), speed);
    });
    
    // Sensor streaming endpoint
    robot.addHttpRoute("/sensors/stream", [](HttpRequest* req, HttpResponse* res, LineRobotBoard* bot) {
        res->setStatusCode(200);
        res->setHeader("Content-Type", "text/event-stream");
        res->setHeader("Cache-Control", "no-cache");
        res->setHeader("Connection", "keep-alive");
        
        // Send sensor data every 100ms for 10 seconds
        for (int i = 0; i < 100; i++) {
            String data = "data: {";
            data += "\"timestamp\":" + String(millis()) + ",";
            data += "\"sensors\":[";
            for (int s = 0; s < 4; s++) {
                data += String(bot->getIRSensorValue(s));
                if (s < 3) data += ",";
            }
            data += "],";
            data += "\"motors\":[" + String(bot->getMotorSpeedLeft()) + "," + 
                    String(bot->getMotorSpeedRight()) + "]";
            data += "}\n\n";
            
            res->print(data);
            delay(100);
        }
    });
    
    // Emergency stop
    robot.addHttpRoute("/emergency", [](HttpRequest* req, HttpResponse* res, LineRobotBoard* bot) {
        bot->emergencyStop();
        res->setStatusCode(200);
        res->print("Emergency stop activated");
    });
    
    // Configuration endpoint
    robot.addHttpRoute("/config", [](HttpRequest* req, HttpResponse* res, LineRobotBoard* bot) {
        if (req->getMethod() == "POST") {
            // Update configuration
            String threshold = req->getParam("threshold");
            if (threshold.length() > 0) {
                bot->setAllIRSensorThresholds(threshold.toInt());
            }
            
            String debug = req->getParam("debug");
            if (debug == "true") {
                bot->setDebug(true);
            } else if (debug == "false") {
                bot->setDebug(false);
            }
            
            res->print("Configuration updated");
        } else {
            // Get configuration
            res->setHeader("Content-Type", "application/json");
            res->printf("{\"debug\":%s,\"uptime\":%lu}", 
                        bot->isDebugEnabled() ? "true" : "false",
                        bot->getUptime());
        }
    });
}

void loop() {
    // Main loop is handled by the robot's internal tasks
    delay(100);
    
    // Optional: Print status periodically
    static unsigned long last_status = 0;
    if (millis() - last_status > 5000) {
        if (robot.isWiFiConnected()) {
            Serial.println("WiFi connected - robot controllable via HTTP");
        } else {
            Serial.println("WiFi not connected");
        }
        last_status = millis();
    }
}
```

---

## Line Following Algorithms

### 6. Edge Following (`edge_follow.ino`)

Follow the edge of a line instead of the center:

```cpp
#include "linerobot_board.h"
#include "linerobot_state.h"

LineRobotState robot_state;
LineRobotBoard robot(&robot_state, Serial);

enum EdgeMode {
    FOLLOW_LEFT_EDGE,
    FOLLOW_RIGHT_EDGE
};

EdgeMode current_mode = FOLLOW_LEFT_EDGE;

void setup() {
    Serial.begin(115200);
    robot.begin();
    
    // Double-press button to switch edge modes
    robot.onBoardButtonDoublePressed([](long time_between) {
        current_mode = (current_mode == FOLLOW_LEFT_EDGE) ? 
                       FOLLOW_RIGHT_EDGE : FOLLOW_LEFT_EDGE;
        
        Serial.println(current_mode == FOLLOW_LEFT_EDGE ? 
                       "Following left edge" : "Following right edge");
        return true;  // Handle the event
    });
}

void loop() {
    if (robot_state.currentState() == LINE_ROBOT_RACING) {
        if (current_mode == FOLLOW_LEFT_EDGE) {
            followLeftEdge();
        } else {
            followRightEdge();
        }
    }
}

void followLeftEdge() {
    // Use left-most sensors (sensors 2 and 3)
    bool inner_triggered = robot.isIRSensorTriggered(2);
    bool outer_triggered = robot.isIRSensorTriggered(3);
    
    int base_speed = 160;
    
    if (inner_triggered && !outer_triggered) {
        // On edge - go straight
        robot.setMotorSpeedBoth(base_speed);
    } else if (inner_triggered && outer_triggered) {
        // Too far right - turn left
        robot.setMotorSpeed(base_speed - 80, base_speed + 40);
    } else if (!inner_triggered && !outer_triggered) {
        // Lost line - turn right to find it
        robot.setMotorSpeed(base_speed + 40, base_speed - 80);
    } else {
        // outer_triggered && !inner_triggered - shouldn't happen normally
        robot.setMotorSpeedBoth(base_speed);
    }
}

void followRightEdge() {
    // Use right-most sensors (sensors 0 and 1)
    bool inner_triggered = robot.isIRSensorTriggered(1);
    bool outer_triggered = robot.isIRSensorTriggered(0);
    
    int base_speed = 160;
    
    if (inner_triggered && !outer_triggered) {
        // On edge - go straight
        robot.setMotorSpeedBoth(base_speed);
    } else if (inner_triggered && outer_triggered) {
        // Too far left - turn right
        robot.setMotorSpeed(base_speed + 40, base_speed - 80);
    } else if (!inner_triggered && !outer_triggered) {
        // Lost line - turn left to find it
        robot.setMotorSpeed(base_speed - 80, base_speed + 40);
    } else {
        // outer_triggered && !inner_triggered - shouldn't happen normally
        robot.setMotorSpeedBoth(base_speed);
    }
}
```

### 7. Junction Handling (`junction_follow.ino`)

Detect and handle line intersections and turns:

```cpp
#include "linerobot_board.h"
#include "linerobot_state.h"

LineRobotState robot_state;
LineRobotBoard robot(&robot_state, Serial);

enum JunctionType {
    NO_JUNCTION,
    T_JUNCTION,
    CROSS_JUNCTION,
    LEFT_TURN,
    RIGHT_TURN,
    END_OF_LINE
};

JunctionType detectJunction() {
    uint8_t sensor_mask = robot.getIRSensorMask();
    
    // Count triggered sensors
    int sensor_count = 0;
    for (int i = 0; i < 4; i++) {
        if (sensor_mask & (1 << i)) sensor_count++;
    }
    
    // Analyze patterns
    if (sensor_count == 0) {
        return END_OF_LINE;
    } else if (sensor_count >= 3) {
        // Three or more sensors - likely a junction
        bool outer_left = sensor_mask & 0x08;   // Sensor 3
        bool outer_right = sensor_mask & 0x01;  // Sensor 0
        
        if (outer_left && outer_right) {
            return CROSS_JUNCTION;
        } else if (outer_left) {
            return LEFT_TURN;
        } else if (outer_right) {
            return RIGHT_TURN;
        } else {
            return T_JUNCTION;
        }
    }
    
    return NO_JUNCTION;
}

void handleJunction(JunctionType type) {
    Serial.print("Junction detected: ");
    
    switch (type) {
        case LEFT_TURN:
            Serial.println("Left turn");
            robot.setMotorSpeed(50, 180);  // Turn left
            delay(300);
            break;
            
        case RIGHT_TURN:
            Serial.println("Right turn");
            robot.setMotorSpeed(180, 50);  // Turn right
            delay(300);
            break;
            
        case T_JUNCTION:
            Serial.println("T-junction - going straight");
            robot.setMotorSpeedBoth(150);
            delay(200);
            break;
            
        case CROSS_JUNCTION:
            Serial.println("Cross junction - going straight");
            robot.setMotorSpeedBoth(150);
            delay(250);
            break;
            
        case END_OF_LINE:
            Serial.println("End of line - stopping");
            robot.emergencyStop();
            robot_state.setState(LINE_ROBOT_IDLE);
            break;
            
        case NO_JUNCTION:
        default:
            // Normal line following
            normalLineFollow();
            break;
    }
}

void normalLineFollow() {
    // Simple proportional control when no junction
    int left = robot.getIRSensorValue(2);
    int right = robot.getIRSensorValue(1);
    int error = right - left;
    
    int base_speed = 180;
    int correction = error * 0.6;
    
    robot.setMotorSpeed(base_speed - correction, base_speed + correction);
}

void setup() {
    Serial.begin(115200);
    robot.begin();
}

void loop() {
    if (robot_state.currentState() == LINE_ROBOT_RACING) {
        JunctionType junction = detectJunction();
        handleJunction(junction);
    }
    
    delay(50);  // Small delay for stability
}
```

---

## Custom HTTP APIs

### 8. RESTful Robot API (`rest_api.ino`)

Create a complete RESTful API for robot control:

```cpp
#include "linerobot_board.h"
#include "linerobot_state.h"

LineRobotState robot_state;
LineRobotBoard robot(&robot_state, Serial, "YourWiFi", "YourPassword");

void setup() {
    Serial.begin(115200);
    robot.begin();
    
    setupRESTAPI();
    
    Serial.println("RESTful Robot API Server Started");
}

void setupRESTAPI() {
    // GET /api/robot - Get robot status
    robot.addHttpRoute("/api/robot", [](HttpRequest* req, HttpResponse* res, LineRobotBoard* bot) {
        res->setHeader("Content-Type", "application/json");
        res->setHeader("Access-Control-Allow-Origin", "*");
        
        String response = "{";
        response += "\"status\":\"" + String(robot_state.currentState()) + "\",";
        response += "\"uptime\":" + String(bot->getUptime()) + ",";
        response += "\"wifi_connected\":" + String(bot->isWiFiConnected() ? "true" : "false") + ",";
        response += "\"motors\":{";
        response += "\"left\":" + String(bot->getMotorSpeedLeft()) + ",";
        response += "\"right\":" + String(bot->getMotorSpeedRight());
        response += "},";
        response += "\"sensors\":[";
        for (int i = 0; i < 4; i++) {
            response += String(bot->getIRSensorValue(i));
            if (i < 3) response += ",";
        }
        response += "],";
        response += "\"triggered_sensors\":" + String(bot->getIRSensorMask());
        response += "}";
        
        res->print(response);
    });
    
    // POST /api/robot/state - Change robot state
    robot.addHttpRoute("/api/robot/state", [](HttpRequest* req, HttpResponse* res, LineRobotBoard* bot) {
        if (req->getMethod() != "POST") {
            res->setStatusCode(405);
            res->print("Method not allowed");
            return;
        }
        
        String new_state = req->getParam("state");
        
        if (new_state == "idle") {
            robot_state.setState(LINE_ROBOT_IDLE);
        } else if (new_state == "primed") {
            robot_state.setState(LINE_ROBOT_PRIMED);
        } else if (new_state == "racing") {
            robot_state.setState(LINE_ROBOT_RACING);
        } else {
            res->setStatusCode(400);
            res->print("Invalid state");
            return;
        }
        
        res->setHeader("Content-Type", "application/json");
        res->printf("{\"status\":\"ok\",\"new_state\":%d}", robot_state.currentState());
    });
    
    // POST /api/motors - Control motors
    robot.addHttpRoute("/api/motors", [](HttpRequest* req, HttpResponse* res, LineRobotBoard* bot) {
        if (req->getMethod() != "POST") {
            res->setStatusCode(405);
            res->print("Method not allowed");
            return;
        }
        
        String left_str = req->getParam("left");
        String right_str = req->getParam("right");
        String both_str = req->getParam("both");
        String ramp_str = req->getParam("ramp_time");
        
        int ramp_time = ramp_str.toInt();
        bool success = false;
        
        if (both_str.length() > 0) {
            int speed = constrain(both_str.toInt(), -255, 255);
            success = bot->setMotorSpeedBoth(speed, ramp_time);
        } else if (left_str.length() > 0 || right_str.length() > 0) {
            int left = constrain(left_str.toInt(), -255, 255);
            int right = constrain(right_str.toInt(), -255, 255);
            success = bot->setMotorSpeed(left, right, ramp_time);
        }
        
        if (success) {
            res->setHeader("Content-Type", "application/json");
            res->printf("{\"status\":\"ok\",\"left\":%d,\"right\":%d}",
                        bot->getMotorSpeedLeft(), bot->getMotorSpeedRight());
        } else {
            res->setStatusCode(400);
            res->print("Invalid motor command");
        }
    });
    
    // POST /api/calibration - Sensor calibration
    robot.addHttpRoute("/api/calibration", [](HttpRequest* req, HttpResponse* res, LineRobotBoard* bot) {
        String action = req->getParam("action");
        
        if (action == "baseline") {
            bool success = bot->baselineIRSensors();
            res->setHeader("Content-Type", "application/json");
            res->printf("{\"status\":\"%s\",\"message\":\"Baseline calibration %s\"}",
                        success ? "ok" : "error",
                        success ? "completed" : "failed");
        } else if (action == "reset") {
            bool success = bot->resetIRBaselines();
            res->setHeader("Content-Type", "application/json");
            res->printf("{\"status\":\"%s\",\"message\":\"Baselines %s\"}",
                        success ? "ok" : "error",
                        success ? "reset" : "reset failed");
        } else {
            res->setStatusCode(400);
            res->print("Invalid calibration action");
        }
    });
    
    // GET /api/diagnostics - Full diagnostic report
    robot.addHttpRoute("/api/diagnostics", [](HttpRequest* req, HttpResponse* res, LineRobotBoard* bot) {
        res->setHeader("Content-Type", "text/plain");
        String diagnostic = bot->performDiagnostic();
        res->print(diagnostic);
    });
}

void loop() {
    // The robot handles everything through its internal tasks
    delay(1000);
}
```

---

## State Machine Programming

### 9. Advanced State Machine (`advanced_states.ino`)

Custom state machine with additional states for complex behaviors:

```cpp
#include "linerobot_board.h"
#include "linerobot_state.h"

// Custom states (extend beyond basic LineRobot states)
#define ROBOT_SEARCHING     10
#define ROBOT_OBSTACLE      11  
#define ROBOT_TURNING       12
#define ROBOT_CALIBRATING   13

LineRobotState robot_state;
LineRobotBoard robot(&robot_state, Serial);

// State timing
unsigned long state_enter_time = 0;
unsigned long last_line_seen = 0;

void setup() {
    Serial.begin(115200);
    robot.begin();
    
    setupStateCallbacks();
}

void setupStateCallbacks() {
    // Standard state callbacks
    robot_state.onIDLEStart([]() {
        Serial.println("State: IDLE");
        robot.setMotorSpeedBoth(0);
        robot.setLEDMask(0x0001);  // Only LED 0 on
    });
    
    robot_state.onPRIMEDStart([]() {
        Serial.println("State: PRIMED - Ready to race!");
        robot.setLEDMask(0x0003);  // LEDs 0,1 on
    });
    
    robot_state.onRACINGStart([]() {
        Serial.println("State: RACING");
        robot.setLEDMask(0x000F);  // LEDs 0-3 on
        last_line_seen = millis();
    });
    
    robot_state.onRACINGEnd([]() {
        robot.setMotorSpeedBoth(0);
    });
    
    // Custom state handling
    robot_state.onStateChange([](uint8_t old_state, uint8_t new_state) {
        state_enter_time = millis();
        
        switch (new_state) {
            case ROBOT_SEARCHING:
                Serial.println("State: SEARCHING for line");
                robot.setLEDMask(0x00AA);  // Alternating pattern
                break;
                
            case ROBOT_OBSTACLE:
                Serial.println("State: OBSTACLE detected");
                robot.setLEDMask(0xFFFF);  // All LEDs on
                break;
                
            case ROBOT_TURNING:
                Serial.println("State: TURNING");
                robot.setLEDMask(0x00F0);  // LEDs 4-7 on
                break;
        }
    });
}

void loop() {
    uint8_t current_state = robot_state.currentState();
    unsigned long state_time = millis() - state_enter_time;
    
    switch (current_state) {
        case LINE_ROBOT_RACING:
            handleRacingState();
            break;
            
        case ROBOT_SEARCHING:
            handleSearchingState(state_time);
            break;
            
        case ROBOT_OBSTACLE:
            handleObstacleState(state_time);
            break;
            
        case ROBOT_TURNING:
            handleTurningState(state_time);
            break;
    }
    
    delay(20);
}

void handleRacingState() {
    // Check if we've lost the line
    uint8_t sensor_mask = robot.getIRSensorMask();
    
    if (sensor_mask != 0) {
        // Line detected - update timestamp and follow
        last_line_seen = millis();
        followLine();
    } else {
        // No line detected - check how long it's been
        if (millis() - last_line_seen > 500) {
            // Lost line for 500ms - start searching
            robot_state.setState(ROBOT_SEARCHING);
        } else {
            // Keep going forward briefly
            robot.setMotorSpeedBoth(100);
        }
    }
}

void handleSearchingState(unsigned long state_time) {
    // Search pattern: spin in place, then move forward
    if (state_time < 1000) {
        // Spin right for 1 second
        robot.setMotorSpeed(150, -150);
    } else if (state_time < 2000) {
        // Move forward for 1 second
        robot.setMotorSpeedBoth(120);
    } else if (state_time < 3000) {
        // Spin left for 1 second
        robot.setMotorSpeed(-150, 150);
    } else {
        // Give up searching - return to idle
        Serial.println("Search failed - returning to idle");
        robot_state.setState(LINE_ROBOT_IDLE);
        return;
    }
    
    // Check if we found the line
    if (robot.getIRSensorMask() != 0) {
        Serial.println("Line found - resuming race");
        robot_state.setState(LINE_ROBOT_RACING);
        last_line_seen = millis();
    }
}

void handleObstacleState(unsigned long state_time) {
    // Obstacle avoidance sequence
    if (state_time < 500) {
        // Back up
        robot.setMotorSpeedBoth(-120);
    } else if (state_time < 1500) {
        // Turn right 
        robot.setMotorSpeed(150, -150);
    } else if (state_time < 2500) {
        // Move forward
        robot.setMotorSpeedBoth(120);
    } else if (state_time < 3500) {
        // Turn left to get back on track
        robot.setMotorSpeed(-150, 150);
    } else {
        // Resume normal operation
        robot_state.setState(LINE_ROBOT_RACING);
    }
}

void handleTurningState(unsigned long state_time) {
    // 90-degree turn sequence
    robot.setMotorSpeed(200, -200);  // Spin right
    
    if (state_time > 800) {  // Turn for 800ms
        robot_state.setState(LINE_ROBOT_RACING);
    }
}

void followLine() {
    // Enhanced line following with obstacle detection
    int left = robot.getIRSensorValue(2);
    int right = robot.getIRSensorValue(1);
    
    // Simple obstacle detection (both outer sensors very high)
    if (robot.getIRSensorValue(0) > 800 && robot.getIRSensorValue(3) > 800) {
        robot_state.setState(ROBOT_OBSTACLE);
        return;
    }
    
    // Proportional control
    int error = right - left;
    int base_speed = 160;
    float kp = 0.8;
    int correction = kp * error;
    
    int left_speed = constrain(base_speed - correction, -255, 255);
    int right_speed = constrain(base_speed + correction, -255, 255);
    
    robot.setMotorSpeed(left_speed, right_speed);
}

// Custom button handler to trigger turns
void setup() {
    // ... existing setup code ...
    
    robot.onBoardButtonLongPressed([](long press_time) {
        if (robot_state.currentState() == LINE_ROBOT_RACING) {
            robot_state.setState(ROBOT_TURNING);
            return true;  // Handle the event
        }
        return false;  // Let default handler run
    });
}
```

---

## Sensor Processing

### 10. Advanced Sensor Processing (`sensor_processing.ino`)

Implement sensor filtering, averaging, and advanced processing:

```cpp
#include "linerobot_board.h"
#include "linerobot_state.h"

LineRobotState robot_state;
LineRobotBoard robot(&robot_state, Serial);

// Moving average filter for sensors
class MovingAverage {
private:
    float* buffer;
    int size;
    int index;
    float sum;
    bool filled;
    
public:
    MovingAverage(int window_size) : size(window_size), index(0), sum(0), filled(false) {
        buffer = new float[size];
        for (int i = 0; i < size; i++) buffer[i] = 0;
    }
    
    ~MovingAverage() {
        delete[] buffer;
    }
    
    float update(float value) {
        sum -= buffer[index];
        buffer[index] = value;
        sum += value;
        
        index = (index + 1) % size;
        if (index == 0) filled = true;
        
        return sum / (filled ? size : index + 1);
    }
    
    float getAverage() const {
        return sum / (filled ? size : (index == 0 && filled) ? size : index);
    }
    
    void reset() {
        for (int i = 0; i < size; i++) buffer[i] = 0;
        index = 0;
        sum = 0;
        filled = false;
    }
};

// Sensor filters
MovingAverage sensor_filters[4] = {
    MovingAverage(5),  // 5-sample moving average
    MovingAverage(5),
    MovingAverage(5), 
    MovingAverage(5)
};

// Accelerometer filter
MovingAverage accel_filters[3] = {
    MovingAverage(10),  // 10-sample for smoother accelerometer
    MovingAverage(10),
    MovingAverage(10)
};

// Edge detection for rapid changes
class EdgeDetector {
private:
    float last_value;
    float threshold;
    bool rising_edge_detected;
    bool falling_edge_detected;
    
public:
    EdgeDetector(float edge_threshold) : threshold(edge_threshold), last_value(0) {
        reset();
    }
    
    void update(float value) {
        float delta = value - last_value;
        
        rising_edge_detected = (delta > threshold);
        falling_edge_detected = (delta < -threshold);
        
        last_value = value;
    }
    
    bool risingEdge() const { return rising_edge_detected; }
    bool fallingEdge() const { return falling_edge_detected; }
    void reset() { rising_edge_detected = falling_edge_detected = false; }
};

EdgeDetector sensor_edges[4] = {
    EdgeDetector(100),  // Threshold for edge detection
    EdgeDetector(100),
    EdgeDetector(100),
    EdgeDetector(100)
};

void setup() {
    Serial.begin(115200);
    robot.begin();
    robot.setDebug(true);
    
    Serial.println("Advanced sensor processing example");
}

void loop() {
    processSensors();
    
    if (robot_state.currentState() == LINE_ROBOT_RACING) {
        advancedLineFollow();
    }
    
    delay(20);
}

void processSensors() {
    // Process IR sensors with filtering
    for (int i = 0; i < 4; i++) {
        float raw_value = robot.getIRSensorValue(i);
        float filtered_value = sensor_filters[i].update(raw_value);
        
        // Update edge detector
        sensor_edges[i].update(filtered_value);
        
        // Log edge events
        if (sensor_edges[i].risingEdge()) {
            Serial.printf("Sensor %d: Rising edge detected\n", i);
        }
        if (sensor_edges[i].fallingEdge()) {
            Serial.printf("Sensor %d: Falling edge detected\n", i);
        }
    }
    
    // Process accelerometer with filtering
    float x, y, z;
    if (robot.getAccelerometerReading(x, y, z)) {
        float filtered_x = accel_filters[0].update(x);
        float filtered_y = accel_filters[1].update(y);
        float filtered_z = accel_filters[2].update(z);
        
        // Detect significant motion changes
        static unsigned long last_motion_log = 0;
        if (millis() - last_motion_log > 1000) {
            Serial.printf("Accelerometer (filtered): X=%.2f, Y=%.2f, Z=%.2f\n",
                          filtered_x, filtered_y, filtered_z);
            last_motion_log = millis();
        }
        
        // Detect if robot is upside down (for auto-calibration)
        if (filtered_z < -0.8) {  // Gravity pointing up = robot upside down
            static bool upside_down_logged = false;
            if (!upside_down_logged) {
                Serial.println("Robot detected upside down - baseline mode available");
                upside_down_logged = true;
            }
        }
    }
}

void advancedLineFollow() {
    // Use filtered sensor values for smoother control
    float sensor_values[4];
    for (int i = 0; i < 4; i++) {
        sensor_values[i] = sensor_filters[i].getAverage();
    }
    
    // Calculate line position using weighted average of filtered values
    float position = calculateFilteredLinePosition(sensor_values);
    
    // Adaptive PID based on sensor confidence
    float confidence = calculateSensorConfidence(sensor_values);
    
    // Adjust control gains based on confidence
    float kp = 1.0 * confidence;  // Lower gain when uncertain
    float base_speed = 150 * confidence;  // Slower when uncertain
    
    float error = position;
    float correction = kp * error;
    
    int left_speed = constrain(base_speed - correction, -255, 255);
    int right_speed = constrain(base_speed + correction, -255, 255);
    
    robot.setMotorSpeed(left_speed, right_speed);
    
    // Debug output
    static unsigned long last_debug = 0;
    if (millis() - last_debug > 200) {
        Serial.printf("Pos: %6.2f, Conf: %4.2f, L: %3d, R: %3d\n",
                      position, confidence, left_speed, right_speed);
        last_debug = millis();
    }
}

float calculateFilteredLinePosition(float sensors[4]) {
    // Calculate weighted position using filtered values
    float numerator = 0;
    float denominator = 0;
    
    float positions[4] = {1.5, 0.5, -0.5, -1.5};  // Right to left
    
    for (int i = 0; i < 4; i++) {
        float value = max(0.0f, sensors[i]);
        numerator += value * positions[i];
        denominator += value;
    }
    
    if (denominator == 0) {
        return 0;  // No line detected
    }
    
    return numerator / denominator;
}

float calculateSensorConfidence(float sensors[4]) {
    // Calculate confidence based on sensor values
    float total_signal = 0;
    float max_signal = 0;
    
    for (int i = 0; i < 4; i++) {
        float value = max(0.0f, sensors[i]);
        total_signal += value;
        if (value > max_signal) max_signal = value;
    }
    
    if (total_signal == 0) return 0.1;  // Very low confidence
    
    // Confidence is higher when signal is strong and well-distributed
    float distribution = total_signal / (max_signal * 4);  // 0-1 range
    float strength = min(1.0f, total_signal / 1000.0f);   // Normalize to 0-1
    
    return min(1.0f, (distribution + strength) / 2.0f);
}
```

---

These examples demonstrate the full range of LineRobot capabilities from simple line following to advanced sensor processing and custom HTTP APIs. Each example builds on the previous concepts and can be combined to create sophisticated robot behaviors.

To use these examples:

1. **Copy the desired example** to a new Arduino sketch
2. **Install the LineRobot library** and dependencies  
3. **Update WiFi credentials** if using network features
4. **Calibrate sensors** using the baseline procedure
5. **Modify parameters** for your specific track and robot setup

The examples are designed to be educational and can be adapted for competition robots, educational projects, or custom applications.
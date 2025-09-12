
#include "linerobot_board.h"
#include "driver/timer.h"
#include <Wire.h>

LineRobotBoard::LineRobotBoard(LineRobotState* state, CachingPrinter& logger, String wifi_ssid, String wifi_password, OledDisplayType oledType, bool debug) {
    this->debug = debug;
    this->logger = &logger;
    this->state = state;
    this->start_millis = millis();
    this->tick1_count = 0;
    this->tick2_count = 0;
    this->onBoardPressedCallback = nullptr;
    this->onBoardDoublePressedCallback = nullptr;
    this->onBoardLongPressedCallback = nullptr;
    this->server = nullptr;

    // Initialise the Serial Line
    Serial.begin(9600);
    // Initialise the I2C Bus
    Wire.begin(15, 16);         // SDA, SCL pins for Robot Board
    Wire.setClock(400000);      // Set the I2C clock speed to 400kHz

    // Wait for a short period for the Serial line to be connected, otherwise, move on...
    while (!Serial && millis() - this->start_millis < 200) {
        delay(50);     // Pause until serial is connected (for a few hundred millis anyway...)
    }
    Serial.println("Line Robot: Initialising");
    

    // Initialise the NVS (EEPROM emulation for ESP32)
    this->eeprom_valid = this->preferences.begin("hubrobot", false);

    // If the ssid has been set, then we can setup the Wifi Manager + HTTP Server
    if (wifi_ssid.length() > 0) {
        // Setup the HTTP Server
        this->server = new HttpServer();
        this->server->setDebug(debug);
        this->server->setLogger(logger);
        this->server->setPort(80);
        this->addDefaultRoutes();
        
        // Setup the Wifi Manager
        this->wifi_man = new WifiManager(wifi_ssid, wifi_password);
        this->wifi_man->onConnected([this](String ip) {
            if (this->debug && this->logger) {
                this->logger->println("Connected to WiFi: " + ip);
            }
            this->oled->setLine(1, ip, true);
            if (this->server) {
                this->server->begin();
            }
        });
        this->wifi_man->onDisconnected([this]() {
            if (this->debug && this->logger) {
                this->logger->println("Disconnected from WiFi");
            }
            this->oled->setLine(1, "<NOTCONNECTED>", true);
        });
    } else {
        this->wifi_man = nullptr;
        this->server = nullptr;
    }

    // Setup the OLED Display (if required)
    this->oled = new HubOLED(oledType);
    this->oled->setHeader("Innovation Hub\nLine Robot");

    // Setup the Shift Register
    this->shift_register = new ShiftRegister(11, 9, 10, 2);  // Data, Clock, Latch, No. of Registers
    
    // Setup the DC Motors
    this->motor_left = new DCMotor(12, 13, 14);
    this->motor_right = new DCMotor(38, 39, 40);

    // Setup the LIS3DH Accelerometer
    this->lis3dh = new LIS3DH(I2C_MODE, 0x19);

    // Setup the Button
    this->board_button = new Button(21, 25, false, true, 1000, 300);
    this->board_button->onPressed(std::bind(&LineRobotBoard::onBoardButtonPressedHandler, this, std::placeholders::_1));
    this->board_button->onDoublePressed(std::bind(&LineRobotBoard::onBoardButtonDoublePressedHandler, this, std::placeholders::_1));
    this->board_button->onLongPressed(std::bind(&LineRobotBoard::onBoardButtonLongPressedHandler, this, std::placeholders::_1));
}

LineRobotBoard::~LineRobotBoard() {
    this->preferences.end();
    
    this->oled->clear(false);
    if (this->wifi_man) {
        this->wifi_man->disconnect();
    }

    delete this->oled;
    if (this->server) {
        delete this->server;
    }
    if (this->wifi_man) {
        delete this->wifi_man;
    }

    delete this->shift_register;
    delete this->motor_left;
    delete this->motor_right;
    // delete this->ads7828;
    delete this->lis3dh;
}

void LineRobotBoard::begin(int ir_threshold) {
    this->logger->println("Line Robot Board: Starting...");
    this->oled->init();
    this->logger->println("Line Robot Board: OLED Initialized");
    
    if (this->wifi_man) {
        this->logger->println("Line Robot Board: Initializing WiFi Manager...");
        this->wifi_man->begin();
    }
    
    this->logger->println("Line Robot Board: Initializing Shift Register...");
    this->initShiftRegister();
    this->logger->println("Line Robot Board: Initializing Motors...");
    this->initMotors();
    this->logger->println("Line Robot Board: Initializing ADC...");
    this->initADC();
    this->logger->println("Line Robot Board: Initializing Accelerometer...");
    this->initAccelerometer();
    this->logger->println("Line Robot Board: Initializing Infrared Sensors...");
    this->initInfraredSensors(ir_threshold);

    // Setup Core Tasks
    this->logger->println("Line Robot Board: Setting up Timers and Tasks...");
    this->setupTimersAndTasks();
    this->logger->println("Line Robot Board: Initialization Complete");
}

void LineRobotBoard::setupTimersAndTasks() {
    timer_config_t timerConfig = {
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_START,
        .intr_type = TIMER_INTR_LEVEL,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = TIMER_AUTORELOAD_EN,
        .divider = 80,
    };
    // Timer0 ticks at 2ms (500Hz)
    timer_init(TIMER_GROUP_0, TIMER_0, &timerConfig);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 2000); // 2ms
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);

    // Timer 1 ticks at 32ms (31.25Hz)
    timer_init(TIMER_GROUP_0, TIMER_1, &timerConfig);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, 32000); // 32ms
    timer_enable_intr(TIMER_GROUP_0, TIMER_1);

    // Create the Core Task for Timer0
    static TaskHandle_t coreTask0 = NULL;
    timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, [](void* arg) -> bool {
    BaseType_t xHigherPriorityTaskWoken;
    if (!coreTask0) {
        // If the task is not created yet, we can't notify it
        // This can happen if the timer interrupt is triggered before the task is created
        return true;
    }
    vTaskNotifyGiveFromISR(coreTask0, &xHigherPriorityTaskWoken);
    return xHigherPriorityTaskWoken == pdTRUE;
    }, this, ESP_INTR_FLAG_IRAM);

    xTaskCreateUniversal([](void* arg) {
        LineRobotBoard* board = (LineRobotBoard*)arg;
        while (true) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);    // Wait for notification from timer
            board->tick1(); // Call the tick1 function
        }
    }, "CoreTask0", 10000, this, 6, &coreTask0, 1); 

    // Create the Core Task for Timer1
    static TaskHandle_t coreTask1 = NULL;
    timer_isr_callback_add(TIMER_GROUP_0, TIMER_1, [](void* arg) -> bool {
        BaseType_t xHigherPriorityTaskWoken;
        if (!coreTask1) {
            // If the task is not created yet, we can't notify it
            // This can happen if the timer interrupt is triggered before the task is created
            return true;
        }
        vTaskNotifyGiveFromISR(coreTask1, &xHigherPriorityTaskWoken);
        return xHigherPriorityTaskWoken == pdTRUE;
    }, this, ESP_INTR_FLAG_IRAM);

    xTaskCreateUniversal([](void* arg) {
        LineRobotBoard* board = (LineRobotBoard*)arg;
        while (true) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);    // Wait for notification from timer
            board->tick2(); // Call the tick2 function
        }
    }, "CoreTask1", 10000, this, 3, &coreTask1, 0);

    timer_start(TIMER_GROUP_0, TIMER_0); // Start timer0
    timer_start(TIMER_GROUP_0, TIMER_1); // Start timer1
}

void LineRobotBoard::pauseTimersAndTasks() {
    timer_pause(TIMER_GROUP_0, TIMER_0);
    timer_pause(TIMER_GROUP_0, TIMER_1);
}

void LineRobotBoard::resumeTimersAndTasks() {
    timer_start(TIMER_GROUP_0, TIMER_0);
    timer_start(TIMER_GROUP_0, TIMER_1);
}

void LineRobotBoard::initAccelerometer() {
    this->lis3dh->settings.adcEnabled = 0;
    this->lis3dh->settings.tempEnabled = 0;
    this->lis3dh->settings.accelSampleRate = 100;   // Hz.  Can be: 0,1,10,25,50,100,200,400,1600,5000 Hz
    this->lis3dh->settings.accelRange = 4;          // Max G force readable.  Can be: 2, 4, 8, 16
    this->lis3dh->settings.xAccelEnabled = 1;       
    this->lis3dh->settings.yAccelEnabled = 1;
    this->lis3dh->settings.zAccelEnabled = 1;
    this->lis3dh->settings.fifoEnabled = 0;         // Disable FIFO
    this->lis3dh->begin();
    this->lis3dh->applySettings();
}

void LineRobotBoard::initADC() {
    // No longer user extrnal ADC, using the internal ADC of the ESP32 instead
}

void LineRobotBoard::initMotors() {
    this->motor_left->stop();
    this->motor_right->stop();
}

void LineRobotBoard::initShiftRegister() {
    this->shift_register->clear();
    // Flash all the LEDs on the shift register
    for (int i = 0; i < 16; i++) {
        this->shift_register->set(i, 1); // Set all bits to 0
        if (i > 0) {
            this->shift_register->set(i-1, 0); // Set previous bit to 0
        }
        delay(100);
    }
    this->shift_register->set(15, 0); // Set the last bit to 0
}

void LineRobotBoard::initInfraredSensors(int ir_threshold) {
    // Set the baseline for each sensor
    int baselines[4] = {0,0,0,0};
    if (this->eeprom_valid) {
        // Read the baselines from the NVS
        for (int i = 0; i < 4; i++) {
            // Load the Baselines from the NVS
            baselines[i] = this->preferences.getInt(("ir_" + String(i)).c_str(), 0  /* default value */);
            if (baselines[i] < -4095 || baselines[i] > 4095) {
                baselines[i] = 0; // Invalid baseline, reset to 0
                this->logger->println("Invalid baseline for IR sensor " + String(i) + ", resetting to 0 [value was " + String(baselines[i]) + "]");
            }
        }
    } else {
        // If the EEPROM is not valid, just set all baselines to 0
        for (int i = 0; i < 4; i++) {
            baselines[i] = 0;
        }
    }

    this->ir_sensors[0] = new InfraredSensor(4, 6, 8, ir_threshold, baselines[0]);
    this->ir_sensors[1] = new InfraredSensor(5, 3, 9, ir_threshold, baselines[1]);
    this->ir_sensors[2] = new InfraredSensor(6, 4, 10, ir_threshold, baselines[2]);
    this->ir_sensors[3] = new InfraredSensor(7, 7, 11, ir_threshold, baselines[3]);
    this->logger->println("IR Sensors Initialised with Baselines: " + String(baselines[0]) + ", " + String(baselines[1]) + ", " + String(baselines[2]) + ", " + String(baselines[3]));

    // Turn on the IR LEDs for each Sensor
    for (int i = 0; i < 4; i++) {
        this->setLED(this->ir_sensors[i]->ir_led_num, true);
    }
}

void LineRobotBoard::tick1() {
    this->tick1_count++;

    // This Loop will tick at ~2ms (500Hz) - this is the main loop for the robot
    // This task runs on the 2nd core of the ESP32, it won't block the main loop + other general ESP32 tasks (like Wifi management etc..)

    // Tick the Infrared Sensors
    if (this->tick1_count & 1 == 1) {  // Only process the infrared sensors every 2nd tick, about 250Hz
        // this->logger->print("Tick1: Ticking Infrared Sensors... ");
        this->tickInfraredSensors();
    }

    if (this->tick1_count % 6 == 0) {  // Only process the accelerometer every 6th tick, about 83.33Hz (and on an even tick to avoid the IR sensor tick)
        // Tick the Accelerometer
        // this->logger->print("Tick1: Ticking Accelerometer... ");
        this->tickAccelerometer();

        // Tick the Shift Register
        // this->logger->print("Tick1: Ticking Shift Register... ");
        this->shift_register->push_updates();   // Will only push the updates if the register state has changed
    }
}

void LineRobotBoard::tick2() {
    this->tick2_count++;

    // This Loop will tick at ~32ms (31.25Hz) - this is the main loop for the robot
    // This task runs on the 1st core of the ESP32, it may block the main loop + other general ESP32 tasks (like Wifi management etc..)
    // It's priority is set to normal, so it may get interrupted by other higher priority tasks

    // Tick the Board Button
    // this->logger->print("Tick2: Ticking Board Button... ");
    this->board_button->tick();
    
    // Process HTTP Connections
    if (this->server && this->tick2_count & 1 == 1) {  // Only process the HTTP server every 2nd tick, about 15.625Hz
        // this->logger->print("Tick2: Ticking HTTP Server... ");
        this->server->tick();
    }
}

void LineRobotBoard::setLED(int led, bool on) {
    if (led < 0 || led > 15) {
        return;
    }
    this->shift_register->set(led, on ? 1 : 0);
}

void LineRobotBoard::setOLEDLine(int line, String text, bool centered) {
    if (line < 0) {
        return;
    }
    this->oled->setLine(line, text, centered);
}

void LineRobotBoard::setMotorSpeedLeft(int speed) {
    if (this->motor_left->getSpeed() == speed) {
        return; // No change in speed
    }

    this->motor_left->setSpeed(speed);
    this->state->motor_speed_left = speed;
    if (this->debug && this->logger) {
        this->logger->println("Set Left Motor Speed: " + String(speed));
    }
}
void LineRobotBoard::setMotorSpeedRight(int speed) {
    if (this->motor_right->getSpeed() == speed) {
        return; // No change in speed
    }
    this->motor_right->setSpeed(speed);
    this->state->motor_speed_right = speed;
    if (this->debug && this->logger) {
        this->logger->println("Set Right Motor Speed: " + String(speed));
    }
}
void LineRobotBoard::setMotorSpeedBoth(int speed) {
    this->setMotorSpeedLeft(speed);
    this->setMotorSpeedRight(speed);
}
void LineRobotBoard::setMotorSpeed(int left, int right) {
    this->setMotorSpeedLeft(left);
    this->setMotorSpeedRight(right);
}

int LineRobotBoard::readAnalogueValue(int pin) {
    return analogRead(pin);
}

void LineRobotBoard::tickInfraredSensors() {
    // This function on average takes 1ms to run (the ADC update is slow)
    for (int i = 0; i < 4; i++) {
        this->ir_sensors[i]->tick(true);
        this->shift_register->set(this->ir_sensors[i]->indicator_led_num, this->ir_sensors[i]->is_triggered, false);
        // Update the state of the IR sensors
        // The IR sensors are in the order: outer right, inner right, inner left, outer left
        switch (i) {
            case 0:
                this->state->ir_outer_right = this->ir_sensors[i]->value;
                this->state->ir_outer_right_triggered = this->ir_sensors[i]->is_triggered;
                break;
            case 1:
                this->state->ir_inner_right = this->ir_sensors[i]->value;
                this->state->ir_inner_right_triggered = this->ir_sensors[i]->is_triggered;
                break;
            case 2:
                this->state->ir_inner_left = this->ir_sensors[i]->value;
                this->state->ir_inner_left_triggered = this->ir_sensors[i]->is_triggered;
                break;
            case 3:
                this->state->ir_outer_left = this->ir_sensors[i]->value;
                this->state->ir_outer_left_triggered = this->ir_sensors[i]->is_triggered;
                break;
        }
    }
}

void LineRobotBoard::
tickAccelerometer() {
    // This function on average takes 1ms to run (the IMU update is slow)
    this->state->pose.updatePose(this->lis3dh->readFloatAccelX(), this->lis3dh->readFloatAccelY(), this->lis3dh->readFloatAccelZ());
}

void LineRobotBoard::addHttpRoute(String route, std::function<void(HttpRequest*, HttpResponse*, LineRobotBoard*)> handler) {
    this->server->on(route, [handler, this](WiFiClient* client, HttpRequest& req) {
        HttpResponse response;
        handler(&req, &response, this);
        return response;
    });
}

unsigned long LineRobotBoard::getCoreTasksTickCount() {
    return this->tick1_count;
}

unsigned long LineRobotBoard::getOtherTasksTickCount() {
    return this->tick2_count;
}

void LineRobotBoard::setDebug(bool debug) {
    this->debug = debug;
    if (this->server) {
        this->server->setDebug(debug);
    }
}

void LineRobotBoard::onBoardButtonPressedHandler(long timePressed) {
    if (this->onBoardPressedCallback) {
        if (this->onBoardPressedCallback(timePressed)) {
            return; // If the callback returns true, then we don't need to do anything else
        }
    }

    if (this->debug && this->logger) {
        this->logger->println("Board Button Pressed: " + String(timePressed));
    }

    if (this->state->currentState() == LINE_ROBOT_IDLE && (this->state->pose.x > 0.80 || this->state->pose.x < -0.80)) {
        // The button was pressed when the robot was IDLE and held mostly vertically
        this->state->setState(LINE_ROBOT_BASELINE_INIT);
        this->preBaseline();
    } else if (this->state->currentState() == LINE_ROBOT_BASELINE_INIT) {
        this->baselineIRSensors();
    } else if (this->state->currentState() == LINE_ROBOT_IDLE) {
        this->state->setState(LINE_ROBOT_PRIMED);
    } else if (this->state->currentState() == LINE_ROBOT_PRIMED) {
        this->state->setState(LINE_ROBOT_RACING);
    } else if (this->state->currentState() == LINE_ROBOT_RACING) {
        this->state->setState(LINE_ROBOT_IDLE);    
    }
}

void LineRobotBoard::onBoardButtonDoublePressedHandler(long timeBetweenPresses) {
    if (this->onBoardDoublePressedCallback) {
        if (this->onBoardDoublePressedCallback(timeBetweenPresses)) {
            return; // If the callback returns true, then we don't need to do anything else
        }
    }

    if (this->state->currentState() == LINE_ROBOT_BASELINE_INIT) {
        // Double press during baseline init will clear the currently saved baseline and return to IDLE state
        pauseTimersAndTasks();
        delay(100);
        for (int i = 0; i < 4; i++) {
            this->ir_sensors[i]->setBaseline(0);                        // Clear the live baseline
            this->preferences.putInt(("ir_" + String(i)).c_str(), 0);   // Clear the saved baseline
        }
        delay(100);
        resumeTimersAndTasks();
        this->state->setState(LINE_ROBOT_IDLE);
    }
    if (this->debug && this->logger) {
        this->logger->println("Board Button Double Pressed: " + String(timeBetweenPresses));
    }
}

void LineRobotBoard::onBoardButtonLongPressedHandler(long timePressed) {
    if (this->onBoardLongPressedCallback) {
        if (this->onBoardLongPressedCallback(timePressed)) {
            return; // If the callback returns true, then we don't need to do anything else
        }
    }

    if (this->debug && this->logger) {
        this->logger->println("Board Button Long Pressed: " + String(timePressed));
    }

    if (this->state->currentState() == LINE_ROBOT_IDLE) {
        this->state->setState(LINE_ROBOT_BASELINE_INIT);
        this->preBaseline();
    } else if (this->state->currentState() == LINE_ROBOT_BASELINE_INIT) {
        // Return to IDLE state
        this->state->setState(LINE_ROBOT_IDLE); // Long press returns to IDLE state
    } else if (this->state->currentState() == LINE_ROBOT_PRIMED) {
        this->state->setState(LINE_ROBOT_IDLE); // Long press returns to IDLE state
    }
}

void LineRobotBoard::onBoardButtonPressed(std::function<bool(long)> callback) {
    this->onBoardPressedCallback = callback;
}
void LineRobotBoard::onBoardButtonDoublePressed(std::function<bool(long)> callback) {
    this->onBoardDoublePressedCallback = callback;
}
void LineRobotBoard::onBoardButtonLongPressed(std::function<bool(long)> callback) {
    this->onBoardLongPressedCallback = callback;
}


void LineRobotBoard::baselineIRSensors() {
    this->oled->setLine(1, "BASELINING", true);
    this->oled->setLine(2, "Do not move", true);
    this->oled->setLine(3, "the robot", true);

    // Ensure the IR LEDs are on
    for (int i = 0; i < 4; i++) {
        this->setLED(this->ir_sensors[i]->ir_led_num, true);
    }

    // Wait for 250ms
    delay(250);

    // Set the baseline for each sensor
    unsigned long start_time = millis();
    int baseline_sums[4] = {0,0,0,0};
    int count = 0;
    while (millis() - start_time < 1000) { // Wait for 2 seconds
        for (int i = 0; i < 4; i++) {
            this->ir_sensors[i]->tick(true); // Force update the sensor value
            baseline_sums[i] += this->ir_sensors[i]->value;
        }
        count++;
        delay(10); // Delay for 10ms
    }
    
    this->oled->setLine(0, "Baseline Complete", true);
    int baselines[4] = {0,0,0,0};
    for (int i = 0; i < 4; i++) {
        int baseline = baseline_sums[i] / count;
        this->oled->setLine(i+1, "IR" + String(i) + ": " + String(baseline), false);
        this->logger->println("IR Sensor " + String(i) + " Baseline: " + String(baseline) + " (avg of " + String(count) + " samples, Sum: " + String(baseline_sums[i]) + ")");
        this->ir_sensors[i]->setBaseline(baseline);
        baselines[i] = baseline;
    }

    // Write the baselines to the EEPROM
    if (this->eeprom_valid) {
        pauseTimersAndTasks();
        delay(100);
        for (int i = 0; i < 4; i++) {
            this->preferences.putInt(("ir_" + String(i)).c_str(), baselines[i]);
        }
        delay(100);
        resumeTimersAndTasks();
        this->logger->println("IR Sensor Baselines saved to NVS: " + String(baselines[0]) + ", " + String(baselines[1]) + ", " + String(baselines[2]) + ", " + String(baselines[3]));
    } else {
        this->logger->println("NVS not valid, will not save IR Sensor Baselines");
    }

    delay(500);

    // Move to IDLE State
    this->state->setState(LINE_ROBOT_IDLE);
}

void LineRobotBoard::preBaseline() {
    this->oled->setLine(1, "Place robot on ground", true);
    this->oled->setLine(2, "OFF the track", true);
    this->oled->setLine(3, "Press Button to Start", true);
}

void LineRobotBoard::setIRSensorThreshold(int sensor, int threshold) {
    if (sensor < 0 || sensor > 3) {
        return;
    }
    this->ir_sensors[sensor]->setThreshold(threshold);
}

int LineRobotBoard::getRawIRSensorValue(int sensor) {
    if (sensor < 0 || sensor > 3) {
        return INVALID_ANALOGUE_CHANNEL; // Invalid channel
    }
    return this->ir_sensors[sensor]->raw_value;
}

void LineRobotBoard::addDefaultRoutes() {
    this->server->on("/", [](WiFiClient* client, HttpRequest& req) {
        HttpResponse response;
        response.status = 200;
        response.body = "<html><head><title>Innovation Hub - Line Robot</title></head><body><h1>Hello!</h1><h3>You're connected to the Line Robot!</h3></body></html>";
        response.headers["Content-Type"] = "text/html";
        return response;
    });

    this->server->on("/set-led", [&](WiFiClient* client, HttpRequest& req) {
        String response_str = "";
        if (req.query.find("all") != req.query.end()) {
            String key = String("all");
            bool on = req.query[key] == "true" || req.query[key] == "1";
            for (int i = 0; i <= 15; i++) {
                this->setLED(i, on);
            }
            response_str += "All LEDs: " + String(on ? "ON" : "OFF") + "\n";
        }
    
        for (int i = 0; i <= 15; i++) {
            String key = String(i);
            if (req.query.find(key) != req.query.end()) {
                bool on = req.query[key] == "true" || req.query[key] == "1";
                this->setLED(i, on);
                response_str += "LED " + key + ": " + String(on ? "ON" : "OFF") + "\n";
            }
        }
    
        HttpResponse resp = HttpResponse();
        resp.status = 200;
        resp.body = response_str;
        resp.headers["Content-Type"] = "text/plain";
        return resp;
    });

    this->server->on("/set-motor", [&](WiFiClient* client, HttpRequest& req) {
        String response_str = "";
        if (req.query.find("left") != req.query.end()) {
            String key = String("left");
            int speed = req.query[key].toInt();
            this->setMotorSpeedLeft(speed);
            response_str += "Left Motor Speed: " + String(speed) + "\n";
        }
    
        if (req.query.find("right") != req.query.end()) {
            String key = String("right");
            int speed = req.query[key].toInt();
            this->setMotorSpeedRight(speed);
            response_str += "Right Motor Speed: " + String(speed) + "\n";
        }
    
        if (req.query.find("both") != req.query.end()) {
            String key = String("both");
            int speed = req.query[key].toInt();
            this->setMotorSpeedBoth(speed);
            response_str += "Both Motors Speed: " + String(speed) + "\n";
        }
    
        HttpResponse resp = HttpResponse();
        resp.status = 200;
        resp.body = response_str;
        resp.headers["Content-Type"] = "text/plain";
        return resp;
    });

    this->server->on("/adc", [&](WiFiClient* client, HttpRequest& req) {
        if (req.jsonRequested()) {
            String response_str = "[\n";
            for (int i = 0; i < 8; i++) {
                int value = this->readAnalogueValue(i);
                if (value == INVALID_ANALOGUE_CHANNEL) {
                    response_str += "  {\"channel\": " + String(i) + ", \"value\": \"Not a valid channel\"}";
                } else {
                    response_str += "  {\"channel\": " + String(i) + ", \"value\": " + String(value) + "}";
                }
                if (i < 7) {
                    response_str += ",\n";
                }
            }
            response_str += "]\n";
            HttpResponse resp = HttpResponse();
            resp.status = 200;
            resp.body = response_str;
            resp.headers["Content-Type"] = "application/json";
            return resp;
        } else {
            String response_str = "ADC Values:\n";
            for (int i = 0; i < 8; i++) {
                int value = this->readAnalogueValue(i);
                if (value == INVALID_ANALOGUE_CHANNEL) {
                    response_str += " [" + String(i) + "]: Not a valid channel\n";
                } else {
                    response_str += " [" + String(i) + "]: " + String(value) + "\n";
                }
            }
        
            HttpResponse resp = HttpResponse();
            resp.status = 200;
            resp.body = response_str;
            resp.headers["Content-Type"] = "text/plain";
            return resp;
        }
    });

    this->server->on("/ir", [&](WiFiClient* client, HttpRequest& req) {
        if (req.jsonRequested()) {
            String response_str = "[\n";
            for (int i = 0; i < 4; i++) {
                response_str += this->ir_sensors[i]->describeJson();
                if (i < 3) {
                    response_str += ",\n";
                }
            }
            response_str += "]\n";
            HttpResponse resp = HttpResponse();
            resp.status = 200;
            resp.body = response_str;
            resp.headers["Content-Type"] = "application/json";
            return resp;
        } else {
            String response_str = "IR Sensor Values:\n";
            for (int i = 0; i < 4; i++) {
                response_str += this->ir_sensors[i]->describe() + "\n";
            }
        
            HttpResponse resp = HttpResponse();
            resp.status = 200;
            resp.body = response_str;
            resp.headers["Content-Type"] = "text/plain";
            return resp;
        }
    });

    this->server->on("/pose", [&](WiFiClient* client, HttpRequest& req) {
        if (req.jsonRequested()) {
            String response_str = "{\n";
            response_str += "  \"x\": " + String(this->state->pose.x) + ",\n";
            response_str += "  \"y\": " + String(this->state->pose.y) + ",\n";
            response_str += "  \"z\": " + String(this->state->pose.z) + ",\n";
            response_str += "  \"speed\": " + String(this->state->pose.speed) + ",\n";
            response_str += "  \"heading\": " + String(this->state->pose.heading) + "\n";
            response_str += "}\n";
            HttpResponse resp = HttpResponse();
            resp.status = 200;
            resp.body = response_str;
            resp.headers["Content-Type"] = "application/json";
            return resp;
        } else {
            String response_str = "Pose:\n";
            response_str += "      X: " + String(this->state->pose.x) + "\n";
            response_str += "      Y: " + String(this->state->pose.y) + "\n";
            response_str += "      Z: " + String(this->state->pose.z) + "\n";
            response_str += "  Speed: " + String(this->state->pose.speed) + "m/s\n";
            response_str += "Heading: " + String(this->state->pose.heading) + " rads\n";
        
            HttpResponse resp = HttpResponse();
            resp.status = 200;
            resp.body = response_str;
            resp.headers["Content-Type"] = "text/plain";
            return resp;
        }
    });
}


InfraredSensor::InfraredSensor(int analog_pin, int indicator_led_num, int ir_led_num, int threshold, int baseline) {
    this->analog_pin = analog_pin;
    this->indicator_led_num = indicator_led_num;
    this->ir_led_num = ir_led_num;
    this->threshold = threshold;
    this->baseline = baseline;
    this->raw_value = 0;
    this->value = 0;
    this->is_triggered = false;
    this->state_changed_in_last_update = false;
    this->time_state_changed = 0;
    this->time_in_last_state = 0;
}

void InfraredSensor::tick(bool channelRequiredUpdate) {
    this->raw_value = analogRead(this->analog_pin);
    this->value = this->raw_value - this->baseline;
    if (this->value > this->threshold) {
        if (!this->is_triggered) {
            this->is_triggered = true;
            this->state_changed_in_last_update = true;
            this->time_in_last_state = millis() - this->time_state_changed;
            this->time_state_changed = millis();
        }
    } else {
        if (this->is_triggered) {
            this->is_triggered = false;
            this->state_changed_in_last_update = true;
            this->time_in_last_state = millis() - this->time_state_changed;
            this->time_state_changed = millis();
        }
    }
}

void InfraredSensor::setThreshold(int threshold) {
    this->threshold = threshold;
}

void InfraredSensor::setBaseline(int baseline) {
    this->baseline = baseline;
}

unsigned long InfraredSensor::timeInLastState() {
    return this->time_in_last_state;
}

unsigned long InfraredSensor::timeInCurrentState() {
    return millis() - this->time_state_changed;
}

String InfraredSensor::describe() {
    String str = "IR Sensor: " + String(this->analog_pin) + "\n";
    str += "       Value: " + String(this->value) + "\n";
    str += "   Triggered: " + String(this->is_triggered) + " (For: " + String(this->timeInCurrentState()) + "ms)\n";
    str += "         Raw: " + String(this->raw_value) + "\n";
    str += "    Baseline: " + String(this->baseline) + "\n";
    str += "   Threshold: " + String(this->threshold) + "\n";
    str += " In Prev For: " + String(this->timeInLastState()) + "ms\n";
    return str;
}

String InfraredSensor::describeJson() {
    String str = "{\n";
    str += "  \"value\": " + String(this->value) + ",\n";
    str += "  \"triggered\": " + String(this->is_triggered) + ",\n";
    str += "  \"raw_value\": " + String(this->raw_value) + ",\n";
    str += "  \"baseline\": " + String(this->baseline) + ",\n";
    str += "  \"threshold\": " + String(this->threshold) + ",\n";
    str += "  \"time_in_last_state\": " + String(this->timeInLastState()) + ",\n";
    str += "  \"time_in_current_state\": " + String(this->timeInCurrentState()) + "\n";
    str += "}\n";
    return str;
}

#include "linerobot_board.h"
#include "driver/timer.h"
#include <Wire.h>
#include <cmath>
#include <algorithm>
#include <esp_task_wdt.h>

LineRobotBoard::LineRobotBoard(LineRobotState* state, CachingPrinter& logger, 
                              const String& wifi_ssid, const String& wifi_password, 
                              OledDisplayType oled_type, bool debug) 
    : logger_(&logger)
    , state_(state)
    , debug_(debug)
    , is_initialised_(false)
    , timers_running_(false)
    , start_millis_(millis())
    , eeprom_valid_(false)
    , core_task0_handle_(nullptr)
    , core_task1_handle_(nullptr)
    , input_voltage_alert_threshold_(0.0)  // Default threshold set to 0 volts (special case for auto tuned threshold)
    , last_voltage_adc_value_(0)
{
    // Validate input parameters
    if (!state) {
        throw std::invalid_argument("LineRobotState cannot be null");
    }
    if (!logger_) {
        throw std::invalid_argument("CachingPrinter cannot be null");
    }

    // Initialise Serial communication
    Serial.begin(9600);  // Whilst a higher baud rate should be used for better performance, it seems to be problematic on the robot board hardware
    
    // Initialise I2C Bus
    Wire.begin(15, 16);         // SDA, SCL pins for Robot Board
    Wire.setClock(400000);      // Set the I2C clock speed to 400kHz

    // Wait briefly for Serial connection
    unsigned long serial_timeout = millis() + 200;
    while (!Serial && millis() < serial_timeout) {
        delay(10);  // Wait for a bit...
    }
    esp_task_wdt_reset();   // Feed the watchdog after waiting for Serial
    logDebug("Line Robot: Initialising...");

    // Initialise NVS (Non-Volatile Storage)
    // eeprom_valid_ = initNVS();
    // No longer doing this here - we'll do it on-demand in load/save configuration functions

    // Load Configuration from NVS
    
    // delay(10);  // Small delay to allow NVS operations to complete
    // esp_task_wdt_reset();   // Feed the watchdog

    // Setup OLED Display first (for status display)
    oled_ = std::unique_ptr<HubOLED>(new HubOLED(oled_type));
    oled_->setHeader("Innovation Hub\nLine Robot v" BOARD_VERSION);

    // Setup networking if credentials provided
    if (wifi_ssid.length() > 0) {
        // Setup HTTP Server
        server_ = std::unique_ptr<HttpServer>(new HttpServer());
        server_->setDebug(debug_);
        server_->setLogger(*logger_);
        server_->setPort(80);
        addDefaultRoutes();
        
        // Setup WiFi Manager
        wifi_man_ = std::unique_ptr<WifiManager>(new WifiManager(wifi_ssid, wifi_password));
        wifi_man_->onConnected([this](String ip) {
            logDebug("Connected to WiFi: " + ip);
            if (oled_) {
                oled_->setLine(1, ip, true);
            }
            if (server_) {
                esp_task_wdt_reset();   // Feed the watchdog before starting the HTTP server
                // Pause timers and tasks
                pauseTimersAndTasks();
                server_->begin();
                resumeTimersAndTasks();
            }
        });
        wifi_man_->onDisconnected([this]() {
            logDebug("Disconnected from WiFi");
            if (oled_) {
                oled_->setLine(1, "<NOT CONNECTED>", true);
            }
        });
    }

    // Initialise hardware components with smart pointers
    shift_register_ = std::unique_ptr<ShiftRegister>(new ShiftRegister(11, 9, 10, 2));  // Data, Clock, Latch, Registers
    motor_left_ = std::unique_ptr<DCMotor>(new DCMotor(12, 13, 14));
    motor_right_ = std::unique_ptr<DCMotor>(new DCMotor(38, 39, 40));
    lis3dh_ = std::unique_ptr<LIS3DH>(new LIS3DH(I2C_MODE, 0x19));

    // Initialise the Segment Display
    voltage_display_ = std::unique_ptr<TM1637Display>(new TM1637Display(41, 42));  // CLK, DIO pins
    voltage_display_->setBrightness(0x08);  // Half Brightness
    voltage_display_->clear();

    // Setup board button with proper binding
    board_button_ = std::unique_ptr<Button>(new Button(21, 25, false, true, 1000, 300));
    board_button_->onPressed([this](long time) { onBoardButtonPressedHandler(time); });
    board_button_->onDoublePressed([this](long time) { onBoardButtonDoublePressedHandler(time); });
    board_button_->onLongPressed([this](long time) { onBoardButtonLongPressedHandler(time); });
}

LineRobotBoard::~LineRobotBoard() {
    // Proper cleanup sequence
    end();  // Stop all timers and tasks first
    
    // Stop motors for safety
    if (motor_left_) {
        motor_left_->stop();
    }
    if (motor_right_) {
        motor_right_->stop();
    }
    
    // Clear display
    if (oled_) {
        oled_->clear(false);
    }
    
    // Disconnect WiFi
    if (wifi_man_) {
        wifi_man_->disconnect();
    }
    
    // Clear all LEDs
    if (shift_register_) {
        shift_register_->clear();
    }
    
    logDebug("Line Robot: Shutdown complete");
    
    // Smart pointers will automatically clean up their resources
}

bool LineRobotBoard::begin(uint16_t ir_threshold) {
    if (is_initialised_) {
        logError("Board already initialised");
        return false;
    }
    
    logDebug("Line Robot Board: Starting initialization...");
    
    // Feed the watchdog to prevent timeout during initialization
    esp_task_wdt_reset();

    
    // Initialise OLED first for status display
    oled_->init();
    oled_->setLine(0, "Initializing...", true);
    logDebug("OLED Initialised");
    
    // Feed watchdog after OLED init
    esp_task_wdt_reset();

    // Finally, Initialise WiFi if configured
    if (wifi_man_) {
        logDebug("Initialising WiFi Manager...");
        oled_->setLine(1, "WiFi starting...", true);
        wifi_man_->begin();
    }

    // Feed watchdog after WiFi init
    esp_task_wdt_reset();

    // Initialise hardware components
    oled_->setLine(2, "Hardware init...", true);
    
    if (!initShiftRegister()) {
        logError("Failed to initialise shift register");
        return false;
    }
    logDebug("Shift Register Initialised");

    esp_task_wdt_reset();  // Feed watchdog after shift register init

    if (!loadConfiguration()) {
        logError("Failed to load configuration from NVS - will run with defaults");
    }
    esp_task_wdt_reset();  // Feed watchdog after NVS Config load

    if (!initMotors()) {
        logError("Failed to initialise motors");
        return false;
    }
    logDebug("Motors Initialised");
    
    if (!initADC()) {
        logError("Failed to initialise ADC");
        return false;
    }
    logDebug("ADC Initialised");
    
    esp_task_wdt_reset();  // Feed watchdog after ADC init
    
    if (!initAccelerometer()) {
        logError("Failed to initialise accelerometer");
        return false;
    }
    logDebug("Accelerometer Initialised");
    
    if (!initInfraredSensors(ir_threshold)) {
        logError("Failed to initialise IR sensors");
        return false;
    }
    logDebug("Infrared Sensors Initialised");

    esp_task_wdt_reset();  // Feed watchdog after IR sensors init

    // Setup core tasks
    oled_->setLine(3, "Starting tasks...", true);
    delay(8);  // Brief delay to allow previous operations to settle before starting the timers + tasks
    if (!setupTimersAndTasks()) {
        logError("Failed to setup timers and tasks");
        return false;
    }
    logDebug("Timers and Tasks Initialised");
        
    is_initialised_ = true;
    oled_->setLine(1, "Ready!", true);
    oled_->setLine(3, "", true);  // Clear status line

    logDebug("Line Robot Board: Initialisation Complete");
    delay(8); // Brief delay to allow everything to settle (and also allow the user to albeit briefly see the "Ready!" message on the OLED)
    return true;
}

void LineRobotBoard::end() {
    if (!is_initialised_) {
        return;
    }
    
    logDebug("Line Robot Board: Shutting down...");
    
    // Clean up timers and tasks
    cleanupTimersAndTasks();
    
    is_initialised_ = false;
    logDebug("Line Robot Board: Shutdown complete");
}

bool LineRobotBoard::setupTimersAndTasks() {
    if (timers_running_) {
        logError("Timers already running");
        return false;
    }
    
    // Feed the watchdog to prevent timeout during initialization
    esp_task_wdt_reset();
    
    // Configure timer settings
    timer_config_t timer_config = {
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_PAUSE,  // Start paused
        .intr_type = TIMER_INTR_LEVEL,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = TIMER_AUTORELOAD_EN,
        .divider = 80,  // 80MHz / 80 = 1MHz timer resolution
    };
    
    // Initialise Timer0 for high-frequency task (2kHz, 500us)
    esp_task_wdt_reset();  // Feed watchdog before timer setup
    if (timer_init(TIMER_GROUP_0, TIMER_0, &timer_config) != ESP_OK) {
        logError("Failed to initialise Timer0");
        return false;
    }
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 500); // 500us in microseconds
    
    // Initialise Timer1 for low-frequency task (62.5Hz, 16ms)
    esp_task_wdt_reset();  // Feed watchdog before timer setup
    if (timer_init(TIMER_GROUP_0, TIMER_1, &timer_config) != ESP_OK) {
        logError("Failed to initialise Timer1");
        return false;
    }
    timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, 16000); // 16ms in microseconds

    // Feed watchdog before task creation (memory allocation can be slow)
    esp_task_wdt_reset();
    
    // Create high-priority task for sensor processing (Core 1)
    BaseType_t result = xTaskCreatePinnedToCore(
        [](void* param) {
            LineRobotBoard* board = static_cast<LineRobotBoard*>(param);
            while (true) {
                // Wait for timer notification with timeout for safety
                if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10)) > 0) {
                    board->tick1();
                }
            }
        },
        "CoreTask0",     // Task name
        8192,            // Stack size
        this,            // Parameter
        6,               // Priority (high)
        &core_task0_handle_,  // Task handle
        1                // Core 1
    );
    
    if (result != pdPASS) {
        logError("Failed to create CoreTask0");
        return false;
    }

    // Feed watchdog before second task creation
    esp_task_wdt_reset();
    
    // Create lower-priority task for misc operations (Core 0)
    result = xTaskCreatePinnedToCore(
        [](void* param) {
            LineRobotBoard* board = static_cast<LineRobotBoard*>(param);
            while (true) {
                // Wait for timer notification with timeout for safety
                if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50)) > 0) {
                    board->tick2();
                }
            }
        },
        "CoreTask1",            // Task name
        12288,                  // Stack size
        this,                   // Parameter
        3,                      // Priority (medium)
        &core_task1_handle_,    // Task handle
        0                       // Core 0
    );
    
    if (result != pdPASS) {
        logError("Failed to create CoreTask1");
        vTaskDelete(core_task0_handle_);
        core_task0_handle_ = nullptr;
        return false;
    }

    // Feed watchdog before ISR setup
    esp_task_wdt_reset();
    
    // Set up timer ISR callbacks
    timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, [](void* arg) -> bool {
        LineRobotBoard* board = static_cast<LineRobotBoard*>(arg);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (board->core_task0_handle_) {
            vTaskNotifyGiveFromISR(board->core_task0_handle_, &xHigherPriorityTaskWoken);
        }
        return xHigherPriorityTaskWoken == pdTRUE;
    }, this, ESP_INTR_FLAG_IRAM);

    timer_isr_callback_add(TIMER_GROUP_0, TIMER_1, [](void* arg) -> bool {
        LineRobotBoard* board = static_cast<LineRobotBoard*>(arg);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (board->core_task1_handle_) {
            vTaskNotifyGiveFromISR(board->core_task1_handle_, &xHigherPriorityTaskWoken);
        }
        return xHigherPriorityTaskWoken == pdTRUE;
    }, this, ESP_INTR_FLAG_IRAM);

    // Feed watchdog before enabling interrupts
    esp_task_wdt_reset();
    
    // Enable timer interrupts
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_enable_intr(TIMER_GROUP_0, TIMER_1);

    // Start timers
    timer_start(TIMER_GROUP_0, TIMER_0);
    timer_start(TIMER_GROUP_0, TIMER_1);
    
    // Final watchdog feed for the tasks setup
    esp_task_wdt_reset();
    
    timers_running_ = true;
    logDebug("Timers and tasks started successfully");
    return true;
}

void LineRobotBoard::cleanupTimersAndTasks() {
    if (!timers_running_) {
        return;
    }
    
    logDebug("Cleaning up timers and tasks...");
    
    // Stop and deinitialise timers
    timer_pause(TIMER_GROUP_0, TIMER_0);
    timer_pause(TIMER_GROUP_0, TIMER_1);
    timer_disable_intr(TIMER_GROUP_0, TIMER_0);
    timer_disable_intr(TIMER_GROUP_0, TIMER_1);
    timer_deinit(TIMER_GROUP_0, TIMER_0);
    timer_deinit(TIMER_GROUP_0, TIMER_1);
    
    // Delete tasks
    if (core_task0_handle_) {
        vTaskDelete(core_task0_handle_);
        core_task0_handle_ = nullptr;
    }
    if (core_task1_handle_) {
        vTaskDelete(core_task1_handle_);
        core_task1_handle_ = nullptr;
    }
    
    timers_running_ = false;
    logDebug("Timers and tasks cleaned up");
}

void LineRobotBoard::pauseTimersAndTasks() {
    if (timers_running_) {
        timer_pause(TIMER_GROUP_0, TIMER_0);
        timer_pause(TIMER_GROUP_0, TIMER_1);
        delay(16); // Give some time before moving on
    }
}

void LineRobotBoard::resumeTimersAndTasks() {
    if (timers_running_) {
        delay(16); // Ensure prior operations have settled
        timer_start(TIMER_GROUP_0, TIMER_0);
        timer_start(TIMER_GROUP_0, TIMER_1);
    }
}

bool LineRobotBoard::initNVS() {
    bool success = preferences_.begin("hubrobot", false);
    if (success) {
        logDebug("NVS initialised successfully");
        delay(32); // Brief delay to allow NVS to stabilise
    } else {
        logError("Failed to initialise NVS");
    }
    return success;
}

bool LineRobotBoard::closeNVS() {
    if (eeprom_valid_) {
        preferences_.end();
        logDebug("NVS closed successfully");
        return true;
    }
    return false;
}

bool LineRobotBoard::initAccelerometer() {
    if (!lis3dh_) {
        logError("Accelerometer object not created");
        return false;
    }
    
    // Configure accelerometer settings
    lis3dh_->settings.adcEnabled = 0;
    lis3dh_->settings.tempEnabled = 0;
    lis3dh_->settings.accelSampleRate = 100;   // 100Hz sample rate
    lis3dh_->settings.accelRange = 4;          // ±4g range
    lis3dh_->settings.xAccelEnabled = 1;       
    lis3dh_->settings.yAccelEnabled = 1;
    lis3dh_->settings.zAccelEnabled = 1;
    lis3dh_->settings.fifoEnabled = 0;         // Disable FIFO for simplicity
    
    // Initialise and apply settings
    uint16_t result = lis3dh_->begin();
    if (result != 0) {
        logError("Failed to initialise LIS3DH accelerometer: " + String(result));
        return false;
    }
    
    lis3dh_->applySettings();
    logDebug("Accelerometer initialised with range ±4g at 100Hz");
    return true;
}

bool LineRobotBoard::initADC() {
    // Configure ADC for 12-bit resolution and appropriate attenuation
    analogReadResolution(12);  // 0-4095 range
    analogSetAttenuation(ADC_11db);  // For 0-3.3V input range
    
    // Test ADC functionality on a known pin
    uint16_t test_reading = analogRead(4);  // Test on IR sensor pin
    if (test_reading > MAX_ANALOG_VALUE) {
        logError("ADC test reading out of range: " + String(test_reading));
        return false;
    }
    
    logDebug("ADC initialised, test reading: " + String(test_reading));
    return true;
}

bool LineRobotBoard::initMotors() {
    if (!motor_left_ || !motor_right_) {
        logError("Motor objects not created");
        return false;
    }
    
    // Stop motors for safety
    motor_left_->stop();
    motor_right_->stop();
    
    // Reset state
    if (state_) {
        state_->motor_speed_left = 0;
        state_->motor_speed_right = 0;
    }
    
    logDebug("Motors initialised and stopped");
    return true;
}

bool LineRobotBoard::initShiftRegister() {
    if (!shift_register_) {
        logError("Shift register object not created");
        return false;
    }
    
    // Clear all LEDs initially
    shift_register_->clear();
    
    // Perform LED test sequence (with feeding of the watchdog so we don't trip it)
    for (uint8_t i = 0; i < 16; i++) {
        shift_register_->set(i, 1);
        if (i > 0) {
            shift_register_->set(i - 1, 0);
        }
        delay(30);  // Delayed to allow visual confirmation
        
        // Feed watchdog every 4 LEDs to prevent timeout
        if (i % 4 == 0) {
            esp_task_wdt_reset();
        }
    }
    shift_register_->set(15, 0);  // Turn off last LED
    
    logDebug("Shift register initialised with LED test");
    return true;
}

bool LineRobotBoard::initInfraredSensors(uint16_t ir_threshold) {
    // IR sensor configuration: [analog_pin, indicator_led, ir_led]
    const uint8_t sensor_configs[NUM_IR_SENSORS][3] = {
        {4, 3, 8},   // Outer right
        {5, 6, 9},   // Inner right  
        {6, 7, 10},  // Inner left
        {7, 4, 11}   // Outer left
    };

    // Initialise IR sensors with smart pointers
    for (uint8_t i = 0; i < NUM_IR_SENSORS; i++) {
        try {
            ir_sensors_[i] = std::unique_ptr<InfraredSensor>(
                new InfraredSensor(
                    sensor_configs[i][0],   // analog_pin
                    sensor_configs[i][1],   // indicator_led_num
                    sensor_configs[i][2],   // ir_led_num
                    ir_threshold,           // threshold
                    config_.ir_baselines[i] // baseline
                )
            );
        } catch (const std::exception& e) {
            logError("Failed to create IR sensor " + String(i) + ": " + String(e.what()));
            return false;
        }
        
        // Turn on IR LED for this sensor
        if (!setLED(sensor_configs[i][2], true)) {
            logError("Failed to turn on IR LED for sensor " + String(i));
            return false;
        }
    }
    
    logDebug("IR Sensors initialised with baselines: " + 
             String(config_.ir_baselines[0]) + ", " + String(config_.ir_baselines[1]) + ", " + 
             String(config_.ir_baselines[2]) + ", " + String(config_.ir_baselines[3]));
    
    return true;
}

void LineRobotBoard::tick1() {
    tick1_count_++;

    // High-frequency sensor processing (2kHz)
    // This task runs on Core1 for dedicated sensor processing

    // Process IR sensors at 1kHz (every tick)
    if (tick1_count_ % 2 == 0) {
        tickInfraredSensors();
    }

    // Process the motor ramping at 500Hz (every 4th tick)
    if (tick1_count_ % 4 == 0) {
        int16_t current_left_speed = getMotorSpeedLeft();
        if (target_motor_speed_left_ != current_left_speed) {
            rampMotorSpeed(true, target_motor_speed_left_, current_left_speed, ramp_speed_diff_per_ms_left_);
        }
        int16_t current_right_speed = getMotorSpeedRight();
        if (target_motor_speed_right_ != current_right_speed) {
            rampMotorSpeed(false, target_motor_speed_right_, current_right_speed, ramp_speed_diff_per_ms_right_);
        }
    }

    // Process accelerometer at ~62.5Hz (every 16th tick) 
    if (tick1_count_ % 16 == 0) {
        tickAccelerometer();
        
        // Update shift register (only if changes are pending)
        if (shift_register_) {
            shift_register_->push_updates();
        }
    }

    // Every 60000 ticks (60 seconds), reset tick counter to avoid overflow and also get the stack watermark
    if (tick1_count_ >= 60000) {
        tick1_count_ = 0;

        // Check stack watermark for CoreTask0
        if (core_task0_handle_) {
            UBaseType_t stackWaterMark = uxTaskGetStackHighWaterMark(core_task0_handle_);
            if (stackWaterMark < 256) {
                logError("WARNING: Core 1 (Priority tasks) has had a LOW stack availablility: " + String(stackWaterMark));
            }
        }
    }
}

void LineRobotBoard::tick2() {
    tick2_count_++;

    // Lower-frequency miscellaneous tasks (62.5Hz)
    // This task runs on Core0 and shares resources with WiFi/system tasks

    // Process board button every tick
    if (board_button_) {
        board_button_->tick();
    }
    if (alt_board_button_) {
        alt_board_button_->tick();
    }
    
    // Process HTTP connections at ~32Hz (every 2nd tick) with error protection
    if (server_ && (tick2_count_ % 2 == 0)) {
        // Check available stack space before processing HTTP
        UBaseType_t stackWaterMark = uxTaskGetStackHighWaterMark(nullptr);
        if (stackWaterMark > 512) { // Only process if we have sufficient stack
            try {
                server_->tick();
            } catch (...) {
                logError("HTTP server tick exception caught");
            }
        } else {
            logError("Low stack in HTTP processing: " + String(stackWaterMark));
        }
    }

    // Read Voltage Iput at ~1Hz (every 62nd tick)
    if (tick2_count_ % 62 == 0) {
        float voltage = getInputVoltageLevel();
        // Update the voltage display
        if (voltage_display_) {
            voltage_display_->showNumberDec(uint32_t(voltage), true, 6, 3);
        }

        if (input_voltage_alert_threshold_ < 1.0) {
            input_voltage_alert_threshold_ += 0.1;
            if (input_voltage_alert_threshold_ > 0.6) {
                input_voltage_alert_threshold_ = voltage - 0.9; // Set threshold to 0.9V below current voltage 
                logDebug("Auto-tuned input voltage alert threshold set to: " + String(input_voltage_alert_threshold_) + "v");
            }
        }
        
        if (voltage < input_voltage_alert_threshold_) {
            // Turn on the board LED to indicate low voltage (GPIO2)
            pinMode(2, OUTPUT);
            if (digitalRead(2) != HIGH) {
                digitalWrite(2, HIGH);
                logError("Input voltage low: " + String(voltage) + "v");
            } else {
                digitalWrite(2, LOW);
            }
        } else {
            // Turn off the board LED if voltage is above threshold
            digitalWrite(2, LOW);
        }
    }

    // Every 3600 ticks (~60 seconds), reset tick counter to avoid overflow
    if (tick2_count_ >= 3600) {
        tick2_count_ = 0;
        // Check stack watermark for CoreTask1
        if (core_task1_handle_) {
            UBaseType_t stackWaterMark = uxTaskGetStackHighWaterMark(core_task1_handle_);
            if (stackWaterMark < 256) {
                logError("WARNING: Core 0 (Other Tasks) has had a LOW stack availablility: " + String(stackWaterMark));
            }
        }
    }
}

// LED Control Methods
bool LineRobotBoard::setLED(uint8_t led, bool on) {
    if (!validateLEDIndex(led) || !shift_register_) {
        return false;
    }
    shift_register_->set(led, on ? 1 : 0);
    return true;
}

void LineRobotBoard::setLEDMask(uint16_t mask) {
    if (!shift_register_) return;
    
    for (uint8_t i = 0; i < 16; i++) {
        bool led_state = (mask >> i) & 1;
        shift_register_->set(i, led_state ? 1 : 0, false);  // Don't push immediately
    }
    shift_register_->push_updates();  // Push all changes at once
}

uint16_t LineRobotBoard::getLEDMask() const {
    if (!shift_register_) return 0;
    
    uint16_t mask = 0;
    for (uint8_t i = 0; i < 16; i++) {
        if (shift_register_->get(i)) {
            mask |= (1 << i);
        }
    }
    return mask;
}

// OLED Control Methods  
bool LineRobotBoard::setOLEDLine(uint8_t line, const String& text, bool centered) {
    if (!oled_ || line > 5) {
        return false;
    }
    oled_->setLine(line, text, centered);
    return true;
}

void LineRobotBoard::clearOLED() {
    if (oled_) {
        oled_->clear(true);
    }
}

// Motor Control Methods with Thread Safety and Validation
bool LineRobotBoard::setMotorSpeedLeft(int16_t speed, uint16_t ramp_time_ms) {
    return setMotorSpeedInternal(true, speed, ramp_time_ms);
}

bool LineRobotBoard::setMotorSpeedRight(int16_t speed, uint16_t ramp_time_ms) {
    return setMotorSpeedInternal(false, speed, ramp_time_ms);
}

bool LineRobotBoard::setMotorSpeedBoth(int16_t speed, uint16_t ramp_time_ms) {
    return setMotorSpeedLeft(speed, ramp_time_ms) && 
           setMotorSpeedRight(speed, ramp_time_ms);
}

bool LineRobotBoard::setMotorSpeed(int16_t left, int16_t right, uint16_t ramp_time_ms) {
    return setMotorSpeedLeft(left, ramp_time_ms) && 
           setMotorSpeedRight(right, ramp_time_ms);
}

int16_t LineRobotBoard::getMotorSpeedLeft() const {
    std::lock_guard<std::mutex> lock(motor_mutex_);
    return motor_left_ ? motor_left_->getSpeed() : 0;
}

int16_t LineRobotBoard::getMotorSpeedRight() const {
    std::lock_guard<std::mutex> lock(motor_mutex_);
    return motor_right_ ? motor_right_->getSpeed() : 0;
}

void LineRobotBoard::emergencyStop() {
    std::lock_guard<std::mutex> lock(motor_mutex_);
    
    if (motor_left_) {
        motor_left_->stop();
    }
    if (motor_right_) {
        motor_right_->stop();
    }
    
    if (state_) {
        state_->motor_speed_left = 0;
        state_->motor_speed_right = 0;
    }
    
    logDebug("Emergency stop activated");
}

// Analog Reading
uint16_t LineRobotBoard::readAnalogueValue(uint8_t pin) const {
    // Validate pin number for ESP32-S3
    if (pin < 1 || pin > 20) {
        return INVALID_ANALOGUE_CHANNEL;
    }
    
    uint16_t value = analogRead(pin);
    return (value > MAX_ANALOG_VALUE) ? INVALID_ANALOGUE_CHANNEL : value;
}

// Thread-safe motor control helper
bool LineRobotBoard::setMotorSpeedInternal(bool is_left, int16_t speed, uint16_t ramp_time_ms) {
    if (!validateMotorSpeed(speed)) {
        logError("Invalid motor speed: " + String(speed));
        return false;
    }
    
    std::lock_guard<std::mutex> lock(motor_mutex_);
    
    DCMotor* motor = is_left ? motor_left_.get() : motor_right_.get();
    if (!motor) {
        return false;
    }
    
    // Check if speed is already set (avoid unnecessary operations)
    if (motor->getSpeed() == speed) {
        return true;
    }
    
    if (is_left) {
        target_motor_speed_left_ = speed;
        ramp_speed_diff_per_ms_left_ = (ramp_time_ms > 0) ? 
            static_cast<float>(speed - motor->getSpeed()) / ramp_time_ms : 0.0f;
    } else {
        target_motor_speed_right_ = speed;
        ramp_speed_diff_per_ms_right_ = (ramp_time_ms > 0) ? 
            static_cast<float>(speed - motor->getSpeed()) / ramp_time_ms : 0.0f;
    }

    if (ramp_time_ms == 0) {
        // Set the motor speed immediately
        motor->setSpeed(speed);
        // Update state
        if (state_) {
            if (is_left) {
                state_->motor_speed_left = speed;
            } else {
                state_->motor_speed_right = speed;
            }
        }
        logDebug("Set " + String(is_left ? "Left" : "Right") + " Motor Speed: " + String(speed));
    }
    
    last_motor_update_ = millis();
    return true;
}

void LineRobotBoard::rampMotorSpeed(bool is_left, int16_t target_speed, int16_t current_speed, uint16_t ramp_speed_diff_per_ms) {
    if (ramp_speed_diff_per_ms == 0 || target_speed == current_speed) {
        // No ramping needed
        return;
    }
    
    unsigned long now = millis();
    unsigned long elapsed_ms = now - last_motor_update_;

    int16_t speed_change = static_cast<int16_t>(ramp_speed_diff_per_ms * elapsed_ms);
    int16_t new_speed = current_speed + speed_change;
    if ((ramp_speed_diff_per_ms > 0 && new_speed > target_speed) || 
        (ramp_speed_diff_per_ms < 0 && new_speed < target_speed)) {
        new_speed = target_speed;  // Clamp to target speed
    }

    if (is_left) {
        motor_left_->setSpeed(new_speed);
    } else {
        motor_right_->setSpeed(new_speed);
    }

    last_motor_update_ = now;
}

void LineRobotBoard::tickInfraredSensors() {
    // Process all IR sensors
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    
    for (uint8_t i = 0; i < NUM_IR_SENSORS; i++) {
        if (!ir_sensors_[i]) continue;
        
        ir_sensors_[i]->tick(true);  // Force ADC update
        
        // Update indicator LEDs (skip during PRIMED state to avoid distractions)
        if (state_ && state_->currentState() != LINE_ROBOT_PRIMED && shift_register_) {
            shift_register_->set(ir_sensors_[i]->getIndicatorLedNum(), 
                               ir_sensors_[i]->isTriggered(), false);
        }
        
        // Update robot state
        if (state_) {
            int16_t value = ir_sensors_[i]->getValue();
            bool triggered = ir_sensors_[i]->isTriggered();
            
            switch (i) {
                case 0: // Outer right
                    state_->ir_outer_right = value;
                    state_->ir_outer_right_triggered = triggered;
                    break;
                case 1: // Inner right
                    state_->ir_inner_right = value;
                    state_->ir_inner_right_triggered = triggered;
                    break;
                case 2: // Inner left
                    state_->ir_inner_left = value;
                    state_->ir_inner_left_triggered = triggered;
                    break;
                case 3: // Outer left
                    state_->ir_outer_left = value;
                    state_->ir_outer_left_triggered = triggered;
                    break;
            }
        }
    }
}

void LineRobotBoard::tickAccelerometer() {
    if (!lis3dh_ || !state_) return;
    
    // Read accelerometer values efficiently
    float accel_x = lis3dh_->readFloatAccelX();
    float accel_y = lis3dh_->readFloatAccelY();
    float accel_z = lis3dh_->readFloatAccelZ();
    
    // Update pose with new accelerometer data
    state_->pose.updatePose(accel_x, accel_y, accel_z);
}

unsigned long LineRobotBoard::getCoreTasksTickCount() const {
    return tick1_count_;
}

unsigned long LineRobotBoard::getOtherTasksTickCount() const {
    return tick2_count_;
}

void LineRobotBoard::onBoardButtonPressedHandler(long time_pressed) {
    // Check for custom callback first
    logDebug("Board Button Pressed: " + String(time_pressed) + "ms");

    if (on_board_pressed_callback_ && on_board_pressed_callback_(time_pressed)) {
        return;  // Custom handler processed the event
    }

    
    if (!state_) return;

    uint8_t current_state = state_->currentState();
    
    // State-specific button handling
    if (current_state == LINE_ROBOT_IDLE && 
        (state_->pose.x > 0.80f || state_->pose.x < -0.80f)) {
        // Robot held vertically - enter baseline mode
        state_->setState(LINE_ROBOT_BASELINE_INIT);
        preBaseline();
    } else if (current_state == LINE_ROBOT_BASELINE_INIT) {
        // Start baselining process
        baselineIRSensors();
    } else if (current_state == LINE_ROBOT_IDLE) {
        // Enter primed state
        state_->setState(LINE_ROBOT_PRIMED);
    } else if (current_state == LINE_ROBOT_PRIMED) {
        // Start racing
        state_->setState(LINE_ROBOT_RACING);
    } else if (current_state == LINE_ROBOT_RACING) {
        // Return to idle
        state_->setState(LINE_ROBOT_IDLE);
    }
}

void LineRobotBoard::onBoardButtonDoublePressedHandler(long time_between_presses) {
    // Check for custom callback first
    if (on_board_double_pressed_callback_ && 
        on_board_double_pressed_callback_(time_between_presses)) {
        return;  // Custom handler processed the event
    }

    logDebug("Board Button Double Pressed: " + String(time_between_presses) + "ms");

    if (!state_) return;

    if (state_->currentState() == LINE_ROBOT_BASELINE_INIT) {
        // Double press during baseline init clears all baselines
        resetIRBaselines();
        state_->setState(LINE_ROBOT_IDLE);
        
        if (oled_) {
            oled_->setLine(1, "Baselines Cleared", true);
            delay(1000);
            oled_->setLine(1, "", true);
        }
    }
}

void LineRobotBoard::onBoardButtonLongPressedHandler(long time_pressed) {
    // Check for custom callback first  
    if (on_board_long_pressed_callback_ && on_board_long_pressed_callback_(time_pressed)) {
        return;  // Custom handler processed the event
    }

    logDebug("Board Button Long Pressed: " + String(time_pressed) + "ms");

    if (!state_) return;

    uint8_t current_state = state_->currentState();
    
    if (current_state == LINE_ROBOT_IDLE) {
        // Long press from idle enters baseline mode
        state_->setState(LINE_ROBOT_BASELINE_INIT);
        preBaseline();
    } else if (current_state == LINE_ROBOT_BASELINE_INIT || 
               current_state == LINE_ROBOT_PRIMED) {
        // Long press returns to idle from other states
        state_->setState(LINE_ROBOT_IDLE);
    }
}


void LineRobotBoard::onBoardButtonPressed(std::function<bool(long)> callback) {
    on_board_pressed_callback_ = callback;
}
void LineRobotBoard::onBoardButtonDoublePressed(std::function<bool(long)> callback) {
    on_board_double_pressed_callback_ = callback;
}
void LineRobotBoard::onBoardButtonLongPressed(std::function<bool(long)> callback) {
    on_board_long_pressed_callback_ = callback;
}


bool LineRobotBoard::baselineIRSensors() {
    if (!oled_) return false;
    
    oled_->setLine(1, "BASELINING", true);
    oled_->setLine(2, " ", true);
    oled_->setLine(3, "Do not move robot", true);
    oled_->setLine(4, " ", true);
    oled_->setLine(5, " ", true);

    
    logDebug("Starting IR sensor baseline calibration");

    // Ensure all IR LEDs are on
    for (uint8_t i = 0; i < NUM_IR_SENSORS; i++) {
        if (ir_sensors_[i]) {
            setLED(ir_sensors_[i]->getIrLedNum(), true);
        }
    }

    // Wait for sensors to stabilise
    delay(250);

    // Collect baseline samples
    unsigned long start_time = millis();
    std::array<long, NUM_IR_SENSORS> baseline_sums = {0, 0, 0, 0};
    long count = 0;
    
    while (millis() - start_time < BASELINE_SAMPLE_DURATION_MS) {
        for (uint8_t i = 0; i < NUM_IR_SENSORS; i++) {
            if (ir_sensors_[i]) {
                ir_sensors_[i]->tick(true);  // Force ADC update
                baseline_sums[i] += ir_sensors_[i]->getRawValue();
            }
        }
        count++;
        delay(BASELINE_SAMPLE_DELAY_MS);
    }
    
    if (count == 0) {
        logError("No baseline samples collected");
        return false;
    }
    
    // Calculate and apply baselines
    oled_->setLine(0, "Baseline Complete", true);
    std::array<int16_t, NUM_IR_SENSORS> baselines;
    
    for (uint8_t i = 0; i < NUM_IR_SENSORS; i++) {
        baselines[i] = static_cast<int16_t>(baseline_sums[i] / count);
        
        if (ir_sensors_[i]) {
            ir_sensors_[i]->setBaseline(baselines[i]);
        }
        
        oled_->setLine(i + 1, "IR" + String(i) + ": " + String(baselines[i]), false);
        logDebug("IR Sensor " + String(i) + " Baseline: " + String(baselines[i]) + 
                " (avg of " + String(count) + " samples)");
    }

    // Save baselines to NVS
    bool save_success = saveBaselines();
    if (save_success) {
        logDebug("IR Sensor Baselines saved to NVS successfully");
    } else {
        logError("Failed to save IR Sensor Baselines to NVS");
    }
    
    delay(100);

    // Return to IDLE state
    if (state_) {
        state_->setState(LINE_ROBOT_IDLE);
    }
    
    return true;
}

bool LineRobotBoard::setIRSensorBaseline(uint8_t sensor, int16_t baseline) {
    if (!validateSensorIndex(sensor) || !ir_sensors_[sensor]) {
        return false;
    }
    
    ir_sensors_[sensor]->setBaseline(baseline);
    
    // Save to NVS
    if (saveBaselines()) {
        logDebug("IR Sensor " + String(sensor) + " baseline saved to NVS");
        return true;
    } else {
        logError("Failed to save IR Sensor " + String(sensor) + " baseline to NVS");
        return false;
    }
}

void LineRobotBoard::preBaseline() {
    if (!oled_) return;
    
    oled_->setLine(0, "BASELINE MODE", true);
    oled_->setLine(1, "Place robot on ground", true);
    oled_->setLine(2, "OFF the track", true);
    oled_->setLine(3, "Press button to start", true);
    oled_->setLine(4, "Double press = clear", true);
    
    logDebug("Entered baseline preparation mode");
}

void LineRobotBoard::addDefaultRoutes() {
    if (!server_) return;

    server_->on("/", [](HttpRequest& req) {
        HttpResponse response;
        response.status = 200;
        response.body = "<!DOCTYPE html><html><head><title>Innovation Hub - Line Robot v" BOARD_VERSION "</title></head>"
                       "<body><h1>Line Robot Control Interface</h1>"
                       "<h3>Board Version: " BOARD_VERSION "</h3>"
                       "<p><a href='/status'>System Status</a> | <a href='/diagnostic'>Diagnostic</a> | <a href='/debug'>Memory Info</a></p>"
                       "<p><a href='/ir'>IR Sensors</a></p>"
                       "<p><a href='/adc'>ADC Readings</a></p>"
                       "<p><a href='/pose'>Pose Estimation</a></p>"
                       "</body></html>";
        response.headers["Content-Type"] = "text/html";
        return response;
    });

    server_->on("/set-led", [this](HttpRequest& req) {
        String response_str = "";
        
        if (req.query.find("all") != req.query.end()) {
            String key = "all";
            bool on = req.query[key] == "true" || req.query[key] == "1";
            for (uint8_t i = 0; i < 16; i++) {
                setLED(i, on);
            }
            response_str += "All LEDs: " + String(on ? "ON" : "OFF") + "\n";
        }
    
        for (uint8_t i = 0; i < 16; i++) {
            String key = String(i);
            if (req.query.find(key) != req.query.end()) {
                bool on = req.query[key] == "true" || req.query[key] == "1";
                if (setLED(i, on)) {
                    response_str += "LED " + key + ": " + String(on ? "ON" : "OFF") + "\n";
                } else {
                    response_str += "LED " + key + ": ERROR\n";
                }
            }
        }
    
        HttpResponse resp;
        resp.status = 200;
        resp.body = response_str.length() > 0 ? response_str : "No LEDs specified";
        resp.headers["Content-Type"] = "text/plain";
        return resp;
    });

    server_->on("/set-motor", [this](HttpRequest& req) {
        String response_str = "";
        
        if (req.query.find("left") != req.query.end()) {
            int16_t speed = static_cast<int16_t>(req.query["left"].toInt());
            if (setMotorSpeedLeft(speed)) {
                response_str += "Left Motor Speed: " + String(speed) + "\n";
            } else {
                response_str += "Left Motor Speed: ERROR (invalid speed)\n";
            }
        }
    
        if (req.query.find("right") != req.query.end()) {
            int16_t speed = static_cast<int16_t>(req.query["right"].toInt());
            if (setMotorSpeedRight(speed)) {
                response_str += "Right Motor Speed: " + String(speed) + "\n";
            } else {
                response_str += "Right Motor Speed: ERROR (invalid speed)\n";
            }
        }
    
        if (req.query.find("both") != req.query.end()) {
            int16_t speed = static_cast<int16_t>(req.query["both"].toInt());
            if (setMotorSpeedBoth(speed)) {
                response_str += "Both Motors Speed: " + String(speed) + "\n";
            } else {
                response_str += "Both Motors Speed: ERROR (invalid speed)\n";
            }
        }
        
        if (req.query.find("stop") != req.query.end()) {
            emergencyStop();
            response_str += "Emergency Stop Activated\n";
        }
    
        HttpResponse resp;
        resp.status = 200;
        resp.body = response_str.length() > 0 ? response_str : "No motor commands specified";
        resp.headers["Content-Type"] = "text/plain";
        return resp;
    });

    server_->on("/adc", [this](HttpRequest& req) {
        HttpResponse resp;
        resp.status = 200;
        
        // Avoid large string concatenations - build response incrementally
        if (req.jsonRequested()) {
            resp.body = "[";
            uint8_t test_pins[] = {4, 5, 6, 7, 8, 9, 10, 11};
            
            for (uint8_t i = 0; i < 4; i++) { // Reduced from 8 to 4 pins to save memory
                if (i > 0) resp.body += ",";
                uint16_t value = readAnalogueValue(test_pins[i]);
                resp.body += "{\"pin\":" + String(test_pins[i]) + ",";
                
                if (value == INVALID_ANALOGUE_CHANNEL) {
                    resp.body += "\"value\":null,\"status\":\"error\"}";
                } else {
                    resp.body += "\"value\":" + String(value) + ",\"status\":\"ok\"}";
                }
            }
            resp.body += "]";
            resp.headers["Content-Type"] = "application/json";
        } else {
            // Simple text response to minimise memory usage
            resp.body = "ADC Values:\n";
            uint8_t test_pins[] = {4, 5, 6, 7};
            
            for (uint8_t i = 0; i < 4; i++) {
                uint16_t value = readAnalogueValue(test_pins[i]);
                resp.body += "Pin" + String(test_pins[i]) + ":";
                resp.body += (value == INVALID_ANALOGUE_CHANNEL) ? "ERR" : String(value);
                resp.body += "\n";
            }
            resp.headers["Content-Type"] = "text/plain";
        }
        
        return resp;
    });

    server_->on("/ir", [this](HttpRequest& req) {
        HttpResponse resp;
        resp.status = 200;
        
        uint8_t sensor_mask = getIRSensorMask();
        if (req.jsonRequested()) {
            resp.body = "[";
            for (uint8_t i = 0; i < NUM_IR_SENSORS && i < 4; i++) { // Limit to 4 sensors
                if (i > 0) resp.body += ",";
                
                if (ir_sensors_[i]) {
                    // Get minimal sensor data safely
                    bool triggered = isIRSensorTriggered(i);
                    int16_t value = getIRSensorValue(i);
                    resp.body += "{\"id\":" + String(i) + 
                               ",\"triggered\":" + String(triggered ? "true" : "false") + 
                               ",\"value\":" + String(value) + "}";
                } else {
                    resp.body += "{\"id\":" + String(i) + ",\"status\":\"error\"}";
                }
            }
            resp.body += "]";
            resp.headers["Content-Type"] = "application/json";
        } else {
            // Minimal text response
            resp.body = "IR Sensors:\n";
            resp.body += "Mask: 0x" + String(sensor_mask, HEX) + "\n";
            for (uint8_t i = 0; i < NUM_IR_SENSORS; i++) {
                bool triggered = isIRSensorTriggered(i);
                resp.body += "S" + String(i) + ":" + String(triggered ? "ON" : "OFF") + "\n";
            }
            resp.headers["Content-Type"] = "text/plain";
        }
        
        return resp;
    });

    server_->on("/pose", [this](HttpRequest& req) {
        if (!state_) {
            HttpResponse resp;
            resp.status = 500;
            resp.body = "State not available";
            resp.headers["Content-Type"] = "text/plain";
            return resp;
        }
        
        if (req.jsonRequested()) {
            String response_str = "{\n";
            response_str += "  \"x\": " + String(state_->pose.x, 6) + ",\n";
            response_str += "  \"y\": " + String(state_->pose.y, 6) + ",\n";
            response_str += "  \"z\": " + String(state_->pose.z, 6) + ",\n";
            response_str += "  \"speed\": " + String(state_->pose.speed, 6) + ",\n";
            response_str += "  \"heading\": " + String(state_->pose.heading, 6) + ",\n";
            response_str += "  \"last_update\": " + String(state_->pose.last_update_time) + "\n";
            response_str += "}";
            
            HttpResponse resp;
            resp.status = 200;
            resp.body = response_str;
            resp.headers["Content-Type"] = "application/json";
            return resp;
        } else {
            String response_str = "Robot Pose & Motion:\n";
            response_str += "  Acceleration X: " + String(state_->pose.x, 3) + " g\n";
            response_str += "  Acceleration Y: " + String(state_->pose.y, 3) + " g\n";
            response_str += "  Acceleration Z: " + String(state_->pose.z, 3) + " g\n";
            response_str += "  Speed: " + String(state_->pose.speed, 3) + " m/s\n";
            response_str += "  Heading: " + String(state_->pose.heading, 3) + " rad\n";
            response_str += "  Last Update: " + String(state_->pose.last_update_time) + " ms\n";
        
            HttpResponse resp;
            resp.status = 200;
            resp.body = response_str;
            resp.headers["Content-Type"] = "text/plain";
            return resp;
        }
    });

    // Add new status and diagnostic routes
    server_->on("/status", [this](HttpRequest& req) {
        HttpResponse resp;
        resp.status = 200;
        resp.body = getSystemStatus();
        resp.headers["Content-Type"] = "application/json";
        return resp;
    });

    server_->on("/diagnostic", [this](HttpRequest& req) {
        HttpResponse resp;
        resp.status = 200;
        resp.body = performDiagnostic();
        resp.headers["Content-Type"] = "text/plain";
        return resp;
    });
    
    // Simple health check endpoint
    server_->on("/health", [this](HttpRequest& req) {
        HttpResponse resp;
        resp.status = 200;
        resp.body = "OK";
        resp.headers["Content-Type"] = "text/plain";
        return resp;
    });
    
    // Stack monitoring endpoint
    server_->on("/debug", [this](HttpRequest& req) {
        HttpResponse resp;
        resp.status = 200;
        
        UBaseType_t task0Stack = uxTaskGetStackHighWaterMark(core_task0_handle_);
        UBaseType_t task1Stack = uxTaskGetStackHighWaterMark(core_task1_handle_);
        size_t freeHeap = ESP.getFreeHeap();
        size_t minFreeHeap = ESP.getMinFreeHeap();
        

        resp.body = "Core 1 Min Free Stack:" + String(task1Stack) + " (priority tasks on this core)" 
                 + "\nCore 0 Min Free Stack:" + String(task0Stack) + " (other tasks on this core)" +
                   "\n            Free Heap:" + String(freeHeap) + 
                   "\n             Min Heap:" + String(minFreeHeap) +
                   "\n      Time since boot:" + String(getUptime()) + " ms";
        resp.headers["Content-Type"] = "text/plain";
        return resp;
    });

    server_->on("/baseline", [this](HttpRequest& req) {
        HttpResponse resp;
        resp.status = 200;
        
        if (req.query.find("action") != req.query.end()) {
            String action = req.query["action"];
            
            if (action == "start") {
                resp.body = baselineIRSensors() ? "Baseline OK" : "Baseline Failed";
            } else if (action == "reset") {
                resp.body = resetIRBaselines() ? "Reset OK" : "Reset Failed";
            } else {
                resp.body = "Invalid action";
            }
        } else {
            resp.body = "Use ?action=start or ?action=reset";
        }
        
        resp.headers["Content-Type"] = "text/plain";
        return resp;
    });
}

// Utility and Helper Methods
bool LineRobotBoard::validateSensorIndex(uint8_t sensor) const {
    return sensor < NUM_IR_SENSORS;
}

bool LineRobotBoard::validateMotorSpeed(int16_t speed) const {
    return speed >= MIN_MOTOR_SPEED && speed <= MAX_MOTOR_SPEED;
}

bool LineRobotBoard::validateLEDIndex(uint8_t led) const {
    return led < 16;
}

void LineRobotBoard::logDebug(const String& message) const {
    if (debug_ && logger_) {
        logger_->println("[DEBUG] " + message);
    }
}

void LineRobotBoard::logError(const String& message) const {
    if (logger_) {
        logger_->println("[ERROR] " + message);
    }
}

unsigned long LineRobotBoard::getUptime() const {
    return millis() - start_millis_;
}

void LineRobotBoard::setDebug(bool debug) {
    debug_ = debug;
    if (server_) {
        server_->setDebug(debug);
    }
    logDebug("Debug mode " + String(debug ? "enabled" : "disabled"));
}

// IR Sensor Methods
bool LineRobotBoard::setIRSensorThreshold(uint8_t sensor, uint16_t threshold) {
    if (!validateSensorIndex(sensor) || !ir_sensors_[sensor]) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    ir_sensors_[sensor]->setThreshold(threshold);
    return true;
}

void LineRobotBoard::setAllIRSensorThresholds(uint16_t threshold) {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    for (auto& sensor : ir_sensors_) {
        if (sensor) {
            sensor->setThreshold(threshold);
        }
    }
}

uint16_t LineRobotBoard::getRawIRSensorValue(uint8_t sensor) const {
    if (!validateSensorIndex(sensor) || !ir_sensors_[sensor]) {
        return INVALID_ANALOGUE_CHANNEL;
    }
    
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    return ir_sensors_[sensor]->getRawValue();
}

int16_t LineRobotBoard::getIRSensorValue(uint8_t sensor) const {
    if (!validateSensorIndex(sensor) || !ir_sensors_[sensor]) {
        return INT16_MAX;
    }
    
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    return ir_sensors_[sensor]->getValue();
}

bool LineRobotBoard::isIRSensorTriggered(uint8_t sensor) const {
    if (!validateSensorIndex(sensor) || !ir_sensors_[sensor]) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    return ir_sensors_[sensor]->isTriggered();
}

uint8_t LineRobotBoard::getIRSensorMask() const {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    uint8_t mask = 0;
    
    for (uint8_t i = 0; i < NUM_IR_SENSORS; i++) {
        if (ir_sensors_[i] && ir_sensors_[i]->isTriggered()) {
            mask |= (1 << i);
        }
    }
    
    return mask;
}

bool LineRobotBoard::isWiFiConnected() const {
    return wifi_man_ && wifi_man_->isConnected();
}

// Button Management
bool LineRobotBoard::addAlternateBoardButtonPin(uint8_t pin, bool active_low) {
    if (alt_board_button_) {
        logError("Alternate button already configured");
        return false;
    }
    
    try {
        alt_board_button_ = std::unique_ptr<Button>(new Button(pin, 25, false, active_low, 1000, 500));
        alt_board_button_->onPressed([this](long time) { onBoardButtonPressedHandler(time); });
        alt_board_button_->onDoublePressed([this](long time) { onBoardButtonDoublePressedHandler(time); });
        alt_board_button_->onLongPressed([this](long time) { onBoardButtonLongPressedHandler(time); });
        
        logDebug("Alternate board button configured on pin " + String(pin));
        return true;
    } catch (const std::exception& e) {
        logError("Failed to create alternate button: " + String(e.what()));
        return false;
    }
}

// Diagnostic and Status Methods
bool LineRobotBoard::getAccelerometerReading(float& x, float& y, float& z) const {
    if (!lis3dh_) {
        return false;
    }
    
    x = lis3dh_->readFloatAccelX();
    y = lis3dh_->readFloatAccelY();
    z = lis3dh_->readFloatAccelZ();
    
    return true;
}

String LineRobotBoard::performDiagnostic() const {
    String report = "=== LineRobot Diagnostic Report ===\n";
    
    // System Info
    report += "Board Version: " + String(BOARD_VERSION) + "\n";
    report += "   Board Type: " + String(BOARD_TYPE) + "\n";
    report += "       Uptime: " + String(getUptime()) + "ms\n";
    report += "  Initialised: " + String(is_initialised_ ? "YES" : "NO") + "\n";
    report += "   Debug Mode: " + String(debug_ ? "ON" : "OFF") + "\n\n";
    
    // Hardware Status
    report += "--- Hardware Status ---\n";
    report += "          OLED: " + String(oled_ ? "OK" : "FAILED") + "\n";
    report += "Shift Register: " + String(shift_register_ ? "OK" : "FAILED") + "\n";
    report += "    Left Motor: " + String(motor_left_ ? "OK" : "FAILED") + "\n";
    report += "   Right Motor: " + String(motor_right_ ? "OK" : "FAILED") + "\n";
    report += " Accelerometer: " + String(lis3dh_ ? "OK" : "FAILED") + "\n";
    report += "  Board Button: " + String(board_button_ ? "OK" : "FAILED") + "\n";
    report += "    Alt Button: " + String(alt_board_button_ ? "OK" : "NOT CONFIGURED") + "\n";
    report += " Input Voltage: " + String(voltageFromADCValue(last_voltage_adc_value_), 2) + "v\n\n";
    
    // Network Status
    report += "--- Network Status ---\n";
    if (wifi_man_) {
        report += "       WiFi: " + String(isWiFiConnected() ? "CONNECTED" : "DISCONNECTED") + "\n";
        report += "HTTP Server: " + String(server_ ? "RUNNING" : "NOT CONFIGURED") + "\n";
    } else {
        report += "WiFi: NOT CONFIGURED\n";
    }
    report += "\n";
    
    // IR Sensors
    report += "--- IR Sensors ---\n";
    for (uint8_t i = 0; i < NUM_IR_SENSORS; i++) {
        if (ir_sensors_[i]) {
            report += "Sensor " + String(i) + ": " + ir_sensors_[i]->describe() + "\n";
        } else {
            report += "Sensor " + String(i) + ": FAILED\n";
        }
    }
    report += "\n";
    
    // Task Status
    report += "--- Task Status ---\n";
    report += "  Timers Running: " + String(timers_running_ ? "YES" : "NO") + "\n";
    report += " Core Task Ticks: " + String(getCoreTasksTickCount()) + "\n";
    report += "Other Task Ticks: " + String(getOtherTasksTickCount()) + "\n";
    report += "\n";
    
    // Storage
    report += "--- Storage ---\n";
    report += "NVS Valid: " + String(eeprom_valid_ ? "YES" : "NO") + "\n";
    
    return report;
}

String LineRobotBoard::getSystemStatus() const {
    String status = "{\n";
    status += "  \"version\": \"" + String(BOARD_VERSION) + "\",\n";
    status += "  \"type\": \"" + String(BOARD_TYPE) + "\",\n";
    status += "  \"uptime\": " + String(getUptime()) + ",\n";
    status += "  \"initialised\": " + String(is_initialised_ ? "true" : "false") + ",\n";
    status += "  \"debug\": " + String(debug_ ? "true" : "false") + ",\n";
    status += "  \"input_voltage\": " + String(voltageFromADCValue(last_voltage_adc_value_), 2) + ",\n";
    status += "  \"wifi_connected\": " + String(isWiFiConnected() ? "true" : "false") + ",\n";
    status += "  \"core_ticks\": " + String(getCoreTasksTickCount()) + ",\n";
    status += "  \"other_ticks\": " + String(getOtherTasksTickCount()) + ",\n";
    status += "  \"ir_mask\": " + String(getIRSensorMask()) + ",\n";
    status += "  \"motor_left\": " + String(getMotorSpeedLeft()) + ",\n";
    status += "  \"motor_right\": " + String(getMotorSpeedRight()) + "\n";
    status += "}";
    return status;
}

bool LineRobotBoard::resetIRBaselines() {
    for (uint8_t i = 0; i < NUM_IR_SENSORS; i++) {
        if (ir_sensors_[i]) {
            ir_sensors_[i]->setBaseline(0);
        }
    }

    if (saveBaselines()) {
        logDebug("IR baselines reset to zero and saved to NVS");
        return true;
    } else {
        logError("Failed to save reset IR baselines to NVS");
        return false;
    }
}

bool LineRobotBoard::loadConfiguration() {
    delay(10);  // Small delay to ensure system stability before loading
    bool success = loadBaselines();
    // We might add some more configs here in the future - for now, just loading the baselines
    
    if (success) {
        logDebug("Configuration loaded successfully");
    } else {
        logError("Failed to load configuration");
    }
    
    return success;
}

bool LineRobotBoard::saveConfiguration() {
    if (!eeprom_valid_) {
        logError("Cannot save configuration - NVS not available");
        return false;
    }
    
    bool success = saveBaselines();
    // We might add some more configs here in the future - for now, just saving the baselines
    
    if (success) {
        logDebug("Configuration saved successfully");
    } else {
        logError("Failed to save configuration");
    }
    
    return success;
}

void LineRobotBoard::setAutoBaseline(bool enable) {
    // TODO: Implement automatic baseline during primed state
    // This would involve sammpling the IR sensors during the primed state and adjusting them accordingly 
    // Challeng is it would need to be carefully designed to avoid mis-calibration if the robot was placed on the line when in the primed state....
    // For now, will not implement and will think about the best way to do this - and we'll force everyone to manually baseline occasionally instead
    logDebug("Auto baseline " + String(enable ? "enabled" : "disabled") + " (feature not yet implementedm, manual baselining required)");
}

// HTTP Route Management
bool LineRobotBoard::addHttpRoute(const String& route, std::function<void(HttpRequest*, HttpResponse*, LineRobotBoard*)> handler) {
    if (!server_) {
        logError("HTTP server not available");
        return false;
    }
    
    server_->on(route, [handler, this](HttpRequest& req) {
        HttpResponse response;
        handler(&req, &response, this);
        return response;
    });
    
    logDebug("HTTP route added: " + route);
    return true;
}

// Configuration Helper Methods
String LineRobotBoard::getConfigKey(const String& key) const {
    return "lr_" + key;  // Prefix to avoid conflicts
}

bool LineRobotBoard::saveBaselines() {    
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    pauseTimersAndTasks();
    if (!initNVS()) {
        resumeTimersAndTasks();
        return false;
    }

    bool success = true;
    for (uint8_t i = 0; i < NUM_IR_SENSORS; i++) {
        if (ir_sensors_[i]) {
            String key = "ir_" + String(i);
            int16_t baseline = ir_sensors_[i]->getBaseline();
            
            if (preferences_.putInt(key.c_str(), baseline) == 0) {
                success = false;
                logError("Failed to save baseline for sensor " + String(i));
            }
        }
    }

    closeNVS();
    
    resumeTimersAndTasks();
    return success;
}

bool LineRobotBoard::loadBaselines() {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    
    // Pause timers and tasks to ensure safe loading
    pauseTimersAndTasks();

    if (!initNVS()) {
        logError("Cannot load baselines - NVS not available - using default baseline of 0");
        for (uint8_t i = 0; i < NUM_IR_SENSORS; i++) {
            config_.ir_baselines[i] = 0;
        }
        resumeTimersAndTasks();
        return false;
    }

    bool success = true;
    for (uint8_t i = 0; i < NUM_IR_SENSORS; i++) {
        String key = "ir_" + String(i);
        int16_t baseline = preferences_.getInt(key.c_str(), 0);
        
        // Validate baseline range
        if (baseline >= -MAX_ANALOG_VALUE && baseline <= MAX_ANALOG_VALUE) {
            config_.ir_baselines[i] = baseline;
            logDebug("Loaded baseline for sensor " + String(i) + ": " + String(baseline));
        } else {
            logError("Invalid stored baseline for sensor " + String(i) + ": " + String(baseline));
            config_.ir_baselines[i] = 0;
            success = false;
        }
    }

    closeNVS();

    // Resume timers and tasks
    resumeTimersAndTasks();
    
    return success;
}


float LineRobotBoard::getInputVoltageLevel() {
    // Read voltage from ADC pin
    uint16_t adc_value = analogRead(ADC_VOLTAGE_PIN);

    // Apply exponential smoothing to reduce noise
    const float smoothing_factor = 0.3f;
    if (last_voltage_adc_value_ == 0) {
        // First reading, no smoothing needed
        last_voltage_adc_value_ = adc_value;
    } else {
        // Apply smoothing: new_value = alpha * current + (1-alpha) * previous
        last_voltage_adc_value_ = static_cast<uint16_t>(
            smoothing_factor * adc_value + (1.0f - smoothing_factor) * last_voltage_adc_value_
        );
    }
    adc_value = last_voltage_adc_value_;
    return voltageFromADCValue(adc_value);
}

float LineRobotBoard::voltageFromADCValue(uint16_t adc_value) const {
    // Convert ADC value to voltage level (assuming 3.3V reference and voltage divider)
    static const float r1 = 75000.0; // Resistor R1 value in ohms
    static const float r2 = 10000.0; // Resistor R2 value in ohms
    static const float divider_ratio = (r1 + r2) / r2;  // Inverted voltage divider ratio to get actual voltage
    float voltage = (adc_value / 4095.0) * 3.3 * divider_ratio;
    return voltage;
}


// InfraredSensor Implementation
InfraredSensor::InfraredSensor(uint8_t analog_pin, uint8_t indicator_led_num, 
                              uint8_t ir_led_num, uint16_t threshold, int16_t baseline)
    : analog_pin_(analog_pin)
    , indicator_led_num_(indicator_led_num)
    , ir_led_num_(ir_led_num)
    , threshold_(threshold)
    , baseline_(baseline)
    , value_(0)
    , raw_value_(0)
    , is_triggered_(false)
    , state_changed_in_last_update_(false)
    , time_state_changed_(0)
    , time_in_last_state_(0)
{
    // Validate pin number for ESP32-S3 ADC channels
    if (analog_pin < 1 || analog_pin > 20) {
        throw std::invalid_argument(("Invalid analog pin: " + String(analog_pin_)).c_str());
    }
}

void InfraredSensor::tick(bool force_update) {
    // Read raw ADC value
    raw_value_ = analogRead(analog_pin_);
    
    // Calculate baseline-adjusted value
    value_ = raw_value_ - baseline_;
    
    // Determine trigger state
    bool new_triggered_state = (value_ > static_cast<int16_t>(threshold_));
    
    // Update state if changed
    if (new_triggered_state != is_triggered_) {
        unsigned long current_time = millis();
        time_in_last_state_ = current_time - time_state_changed_;
        time_state_changed_ = current_time;
        is_triggered_ = new_triggered_state;
        state_changed_in_last_update_ = true;
    } else {
        state_changed_in_last_update_ = false;
    }
}

void InfraredSensor::setThreshold(uint16_t threshold) {
    threshold_ = threshold;
}

void InfraredSensor::setBaseline(int16_t baseline) {
    baseline_ = baseline;
}

void InfraredSensor::reset() {
    value_ = 0;
    raw_value_ = 0;
    is_triggered_ = false;
    state_changed_in_last_update_ = false;
    time_state_changed_ = millis();
    time_in_last_state_ = 0;
}

unsigned long InfraredSensor::timeInLastState() const {
    return time_in_last_state_;
}

unsigned long InfraredSensor::timeInCurrentState() const {
    return millis() - time_state_changed_;
}

String InfraredSensor::describe() const {
    String str = "IR Sensor Pin: " + String(analog_pin_) + "\n";
    str += "  Adj Value: " + String(value_) + "\n";
    str += "  Triggered: " + String(is_triggered_ ? "YES" : "NO") + 
           " (For: " + String(timeInCurrentState()) + "ms)\n";
    str += "  Raw Value: " + String(raw_value_) + "\n";
    str += "   Baseline: " + String(baseline_) + "\n";
    str += "  Threshold: " + String(threshold_) + "\n";
    str += "Prev State: " + String(timeInLastState()) + "ms\n";
    return str;
}

String InfraredSensor::describeJson() const {
    String str = "{\n";
    str += "  \"pin\": " + String(analog_pin_) + ",\n";
    str += "  \"value\": " + String(value_) + ",\n";
    str += "  \"triggered\": " + String(is_triggered_ ? "true" : "false") + ",\n";
    str += "  \"raw_value\": " + String(raw_value_) + ",\n";
    str += "  \"baseline\": " + String(baseline_) + ",\n";
    str += "  \"threshold\": " + String(threshold_) + ",\n";
    str += "  \"time_in_last_state\": " + String(timeInLastState()) + ",\n";
    str += "  \"time_in_current_state\": " + String(timeInCurrentState()) + "\n";
    str += "}";
    return str;
}
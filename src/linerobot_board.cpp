
#include "linerobot_board.h"
#include "driver/timer.h"
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cmath>
#include <algorithm>
#include <esp_task_wdt.h>
#include "esp_wifi.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

LineRobotBoard::LineRobotBoard(LineRobotState* state, CachingPrinter& logger, 
                              const String& wifi_ssid, const String& wifi_password, 
                              OledDisplayType oled_type, bool debug, bool use_internal_timers) 
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
    , target_motor_speed_left_(0)
    , target_motor_speed_right_(0)
    , ramp_speed_diff_per_ms_left_(0)
    , ramp_speed_diff_per_ms_right_(0)
    , last_voltage_adc_value_(0)
    , last_motor_update_(0)
    , display_indicator_leds_while_racing_(true)
    , motor_driver_input_voltage_(9.0)
    , motor_driver_voltage_drop_(1.4) // 
    , motor_max_voltage_(6)
    , use_internal_timers_(use_internal_timers)
    , ip_address_("")
{
    // Validate input parameters
    if (!state) {
        throw std::invalid_argument("LineRobotState cannot be null");
    }
    if (!logger_) {
        throw std::invalid_argument("CachingPrinter cannot be null");
    }
    
    // Set Pull-Down resistors on 41 and 42 (open GPIO pins on board with attached LEDs)
    gpio_pulldown_en(GPIO_NUM_41);
    gpio_pulldown_en(GPIO_NUM_42);

    // Set Pull-Down resistors on motor control pins to ensure motors are off by default
    gpio_pulldown_en(GPIO_NUM_12);
    gpio_pulldown_en(GPIO_NUM_38);


    // Initialise Serial communication
    Serial.begin(115200);  // High speed for non-blocking logging
    
    // Initialise I2C Bus
    Wire.begin(15, 16);         // SDA, SCL pins for Robot Board
    Wire.setClock(100000);      // The slower 100kHz is more stable on the ESP32S3
    Wire.setTimeOut(10);        // 10ms timeout prevents infinite hangs if bus crashes
    delay(50);  // Allow I2C bus to stabilize
    esp_task_wdt_reset();

    // Wait briefly for Serial connection
    unsigned long serial_timeout = millis() + 200;
    while (!Serial && millis() < serial_timeout) {
        delay(10);  // Wait for a bit...
        esp_task_wdt_reset();   // Feed the watchdog during wait
    }
    esp_task_wdt_reset();   // Feed the watchdog after waiting for Serial
    
    // Logging MUST come after Serial is ready
    logDebug("Line Robot: Initialising...");

    // Initialise NVS (Non-Volatile Storage)
    // eeprom_valid_ = initNVS();
    // No longer doing this here - we'll do it on-demand in load/save configuration functions
    
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
        
        // Setup HTTP Client
        client_ = std::unique_ptr<HubHttpClient>(new HubHttpClient());
        client_->setTimeout(20000);  // 20 second timeout for requests
        client_->setUserAgent("HubLineRobot/" BOARD_VERSION);

        // Setup WiFi Manager
        wifi_man_ = std::unique_ptr<WifiManager>(new WifiManager(wifi_ssid, wifi_password));
        wifi_man_->onConnected([this](String ip) {
            logDebug("Connected to WiFi: " + ip);
            ip_address_ = ip;
            if (oled_) {
                oled_->setLine(1, ip, true);
            }
            if (server_) {
                logDebug("Starting HTTP server...");
                server_->begin();
                logDebug("HTTP server started");
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

    // Setup board button with proper binding
    board_button_ = std::unique_ptr<Button>(new Button(21, 25, false, true, 1000, 300));
    board_button_->onPressed([this](long time) { onBoardButtonPressedHandler(time); });
    board_button_->onDoublePressed([this](long time) { onBoardButtonDoublePressedHandler(time); });
    board_button_->onLongPressed([this](long time) { onBoardButtonLongPressedHandler(time); });

    // Setup Board Button 2 with proper binding
    board_button2_ = std::unique_ptr<Button>(new Button(8, 25, false, true, 1000, 300));
    board_button2_->onPressed([this](long time) { onBoardButton2PressedHandler(time); });
    board_button2_->onDoublePressed([this](long time) { onBoardButton2DoublePressedHandler(time); });
    board_button2_->onLongPressed([this](long time) { onBoardButton2LongPressedHandler(time); });   

}

LineRobotBoard::~LineRobotBoard() {
    // Set the motors to 0 and pause briefly to allow them to ramp down
    setMotorSpeedBoth(0, 32);  // Ramp down over 32ms
    delay(50);  // Wait a bit longer than the ramp time

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
    
    // Disable Brownout Detector to prevent resets when 3.3V rail sags due to thermal throttling
    // WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    
    delay(100); // Just hang 10 for a bit, we're going to be accessing the NVS shortly and that can be slow and trigger issues with the cached memory'
    // Feed the watchdog to prevent timeout during initialization
    esp_task_wdt_reset();

    
    // Initialise OLED first for status display
    {
        std::lock_guard<std::mutex> lock(i2c_mutex_);
        oled_->init();
    }
    oled_->setLine(0, "Initializing...", true);
    logDebug("OLED Initialised");
    
    // Feed watchdog after OLED init
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
    if (!use_internal_timers_) {
        logDebug("Skipping internal timers and tasks setup as per configuration");
        oled_->setLine(2, "Skipping tasks...", true);
    } else {
        oled_->setLine(3, "Starting tasks...", true);
        delay(8);  // Brief delay to allow previous operations to settle before starting the timers + tasks
        if (!setupTimersAndTasks()) {
            logError("Failed to setup timers and tasks");
            return false;
        }
        logDebug("Timers and Tasks Initialised");
            
    }
    is_initialised_ = true;
    oled_->setLine(1, "Ready!", true);
    oled_->setLine(3, "", true);  // Clear status line


    // Finally, Initialise WiFi if configured
    if (wifi_man_) {
        wifi_man_->begin();
    }

    // Feed watchdog after WiFi init
    esp_task_wdt_reset();

    // Set pin mode for onboard LED (pin 2)
    pinMode(2, OUTPUT);

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

// Wrapper functions for FreeRTOS tasks
void coreTask0Wrapper(void* parameter) {
    LineRobotBoard* board = static_cast<LineRobotBoard*>(parameter);
    TickType_t xTicksStart = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1);  // 1ms = 1kHz
    
    while (true) {
        board->tick1();
        xTaskDelayUntil(&xTicksStart, xFrequency);
    }
}

void coreTask1Wrapper(void* parameter) {
    LineRobotBoard* board = static_cast<LineRobotBoard*>(parameter);
    TickType_t xTicksStart = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(16);  // 16ms = 62.5Hz
    
    while (true) {
        board->tick2();
        xTaskDelayUntil(&xTicksStart, xFrequency);
    }
}

bool LineRobotBoard::setupTimersAndTasks() {
    if (timers_running_) {
        logError("Tasks already running");
        return false;
    }
    
    // Feed the watchdog to prevent timeout during initialization
    esp_task_wdt_reset();
    
    // Create high-frequency task (1kHz) on Core 1
    // Stack size: 8192 bytes, Priority: High
    BaseType_t result = xTaskCreatePinnedToCore(
        coreTask0Wrapper,     // Task function
        "CoreTask0",          // Task name
        8192,                 // Stack size (bytes)
        this,                 // Parameter
        6,                    // Priority
        &core_task0_handle_,  // Task handle
        1                     // Core 1
    );
    
    if (result != pdPASS || core_task0_handle_ == nullptr) {
        logError("Failed to create CoreTask0");
        return false;
    }
    
    // Create low-frequency task (62.5Hz) on Core 1
    // Stack size: 8192 bytes (larger for HTTP server operations), Priority: 2
    result = xTaskCreatePinnedToCore(
        coreTask1Wrapper,     // Task function
        "CoreTask1",          // Task name
        10240,                // Stack size (bytes) - increased for HTTP operations
        this,                 // Parameter
        2,                    // Priority
        &core_task1_handle_,  // Task handle
        tskNO_AFFINITY        // Core 1
    );
    
    if (result != pdPASS || core_task1_handle_ == nullptr) {
        logError("Failed to create CoreTask1");
        // Clean up task0 if task1 creation failed
        if (core_task0_handle_ != nullptr) {
            vTaskDelete(core_task0_handle_);
            core_task0_handle_ = nullptr;
        }
        return false;
    }
    
    // Final watchdog feed after tasks setup
    esp_task_wdt_reset();
    
    timers_running_ = true;
    logDebug("FreeRTOS tasks started successfully");
    return true;
}

void LineRobotBoard::cleanupTimersAndTasks() {
    if (!timers_running_) {
        return;
    }
    
    logDebug("Cleaning up FreeRTOS tasks...");
    
    // Delete FreeRTOS tasks
    if (core_task0_handle_ != nullptr) {
        vTaskDelete(core_task0_handle_);
        core_task0_handle_ = nullptr;
    }
    
    if (core_task1_handle_ != nullptr) {
        vTaskDelete(core_task1_handle_);
        core_task1_handle_ = nullptr;
    }
    
    // Small delay to ensure tasks are fully cleaned up
    vTaskDelay(pdMS_TO_TICKS(10));
    
    timers_running_ = false;
    logDebug("FreeRTOS tasks cleaned up");
}

void LineRobotBoard::pauseTimersAndTasks() {
    if (timers_running_) {
        logDebug("Suspending FreeRTOS tasks...");
        if (core_task0_handle_ != nullptr) {
            vTaskSuspend(core_task0_handle_);
        }
        if (core_task1_handle_ != nullptr) {
            vTaskSuspend(core_task1_handle_);
        }
    }
}

void LineRobotBoard::resumeTimersAndTasks() {
    if (timers_running_) {
        logDebug("Resuming FreeRTOS tasks...");
        if (core_task0_handle_ != nullptr) {
            vTaskResume(core_task0_handle_);
        }
        if (core_task1_handle_ != nullptr) {
            vTaskResume(core_task1_handle_);
        }
    }
}

bool LineRobotBoard::initNVS(bool read_only) {
    bool success = preferences_.begin("hbot", false);
    if (success) {
        logDebug("NVS initialised successfully");
        eeprom_valid_ = true;
        delay(32); // Brief delay to allow NVS to stabilise
    } else {
        logError("Failed to initialise NVS");
        eeprom_valid_ = false;
    }
    return success;
}

bool LineRobotBoard::closeNVS() {
    if (eeprom_valid_) {
        preferences_.end();
        logDebug("NVS closed successfully");
        return true;
    }
    logError("NVS was not valid, cannot close");
    return false;
}

bool LineRobotBoard::initAccelerometer() {
    if (!lis3dh_) {
        logError("Accelerometer object not created");
        return false;
    }
    
    // Protect I2C bus access with mutex
    std::lock_guard<std::mutex> lock(i2c_mutex_);
    
    // Configure accelerometer settings
    lis3dh_->settings.adcEnabled = 0;
    lis3dh_->settings.tempEnabled = 0;
    lis3dh_->settings.accelSampleRate = 100;   // 100Hz sample rate
    lis3dh_->settings.accelRange = 4;          // ±4g range
    lis3dh_->settings.xAccelEnabled = 1;       
    lis3dh_->settings.yAccelEnabled = 1;
    lis3dh_->settings.zAccelEnabled = 1;
    lis3dh_->settings.fifoEnabled = 0;         // Disable FIFO for simplicity
    
    // Not calling begin as the library code is attempting to start the I2C bus again
    // Initialise and apply settings
    // uint16_t result = lis3dh_->begin();
    // if (result != 0) {
    //     logError("Failed to initialise LIS3DH accelerometer: " + String(result));
    //     return false;
    // }
    
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

void LineRobotBoard::tick() {
    tick1();
    static unsigned long last_tick2_time = 0;
    if (millis() - last_tick2_time >= 16) {  // Approximately every 16ms
        last_tick2_time = millis();
        tick2();
    }
}

void LineRobotBoard::tick1() {
    tick1_count_++;

    // Get the core ID for logging purposes
    int core_id = xPortGetCoreID();
    static unsigned long last_tick_time = micros();
    static unsigned long tick_interval_sum = 0;
    static unsigned long tick_interval_count = 0;
    static unsigned long tick_duration_sum = 0;
    static unsigned long last_log_time = 0;

    unsigned long start_time = micros();

    // High-frequency sensor processing (1kHz)

    // Process IR sensors at 500Hz (every other tick)
    if (tick1_count_ % 2 == 0) {
        tickInfraredSensors();
    }

    // Process the motor ramping at 250Hz (every 4th tick)
    if (tick1_count_ % 4 == 0) {
        // logDebug("Ramping motors to targets: L=" + String(target_motor_speed_left_) + " R=" + String(target_motor_speed_right_));
        int16_t current_left_speed = getMotorSpeedLeft();
        // logDebug("Current motor speeds: L=" + String(current_left_speed) + " R=" + String(getMotorSpeedRight()));
        if (target_motor_speed_left_ != current_left_speed) {
            rampMotorSpeed(true, target_motor_speed_left_, current_left_speed, ramp_speed_diff_per_ms_left_);
        }
        int16_t current_right_speed = getMotorSpeedRight();
        if (target_motor_speed_right_ != current_right_speed) {
            rampMotorSpeed(false, target_motor_speed_right_, current_right_speed, ramp_speed_diff_per_ms_right_);
        }
    }

    // Process accelerometer at ~125Hz (every 8th tick) 
    if (tick1_count_ % 8 == 0) {
        tickAccelerometer();
    }

    // Update the shift register at 62Hz (every 16th tick)
    if (tick1_count_ % 16 == 0) {
        // Update shift register (only if changes are pending)
        if (shift_register_) {
            shift_register_->push_updates();
        }
    }

    // Every 60000 ticks (60 seconds), reset tick counter to avoid overflow and also get the stack watermark
    // if (tick1_count_ >= 60000) {
    //     tick1_count_ = 0;

    //     // // Check stack watermark for CoreTask0
    //     // if (core_task0_handle_) {
    //     //     UBaseType_t stackWaterMark = uxTaskGetStackHighWaterMark(nullptr);
    //     //     if (stackWaterMark < 256) {
    //     //         logError("WARNING: Core 1 (Priority tasks) has had a LOW stack availablility: " + String(stackWaterMark));
    //     //     }
    //     // }
    // }

    unsigned long end_time = micros();
    tick_interval_sum += start_time - last_tick_time;
    tick_duration_sum += end_time - start_time;
    tick_interval_count++;
    last_tick_time = start_time;
    if (millis() - last_log_time >= 1000) {  // Log every 1000ms (1 second)
        unsigned long tick_interval = tick_interval_sum / tick_interval_count;
        avg_tick1_interval_us_ = tick_interval;
        avg_tick1_duration_us_ = tick_duration_sum / tick_interval_count;

        tick_interval_sum = 0;
        tick_interval_count = 0;
        tick_duration_sum = 0;
        last_log_time = millis();
        // logDebug("Tick1 Avg: " + String(tick_interval) + "us (Core " + String(core_id) + ")");
    }
}

void LineRobotBoard::tick2() {
    tick2_count_++;

    // Lower-frequency miscellaneous tasks (62.5Hz)

        // Get the core ID for logging purposes
    int core_id = xPortGetCoreID();
    static unsigned long last_log_time = 0;
    static unsigned long last_tick_time = micros();
    static unsigned long tick_interval_sum = 0;
    static unsigned long tick_duration_sum = 0;
    static unsigned long tick_interval_count = 0;

    unsigned long start_time = micros();


    // Process board button every tick
    if (board_button_) {
        board_button_->tick();
    }
    if (alt_board_button_) {
        alt_board_button_->tick();
    }

    if (board_button2_) {
        board_button2_->tick();
    }
    if (alt_board_button2_) {
        alt_board_button2_->tick();
    }
    
    // Process HTTP connections at ~32Hz (every 2nd tick) with error protection
    if (server_ && (tick2_count_ % 2 == 0)) {
        // Check available stack space before processing HTTP
        // UBaseType_t stackWaterMark = uxTaskGetStackHighWaterMark(nullptr);
        // if (stackWaterMark > 512) { // Only process if we have sufficient stack
            try {
                if (server_->isRunning()) {
                    server_->tick();
                }
            } catch (...) {
                logError("HTTP server tick exception caught");
            }
        // } else {
            // logError("Low stack in HTTP processing: " + String(stackWaterMark));
        // }
    }

    // Read Voltage Iput at ~1Hz (every 62nd tick)
//     if (tick2_count_ % 62 == 0) {
//         float voltage = getInputVoltageLevel();
//         if (input_voltage_alert_threshold_ < 1.0) {
//             input_voltage_alert_threshold_ += 0.1;
//             if (input_voltage_alert_threshold_ > 0.6) {
//                 input_voltage_alert_threshold_ = voltage - 0.9; // Set threshold to 0.9V below current voltage 
//                 logDebug("Auto-tuned input voltage alert threshold set to: " + String(input_voltage_alert_threshold_) + "v");
//             }
//         }
        
//         if (voltage < input_voltage_alert_threshold_) {
//             // Feed the watchdog
//             esp_task_wdt_reset();
//             // Turn on the board LED to indicate low voltage (GPIO2)
//             pinMode(2, OUTPUT);
//             if (digitalRead(2) != HIGH) {
//                 digitalWrite(2, HIGH);
//                 logError("Input voltage low: " + String(voltage) + "v");
//             } else {
//                 digitalWrite(2, LOW);
//             }
//         } else {
//             // Turn off the board LED if voltage is above threshold
//             digitalWrite(2, LOW);
//         }
//     }

    // Every 3600 ticks (~60 seconds), reset tick counter to avoid overflow
    // if (tick2_count_ >= 3600) {
    //     tick2_count_ = 0;
    //     // Check stack watermark for CoreTask1
    //     // if (core_task1_handle_) {
    //     //     UBaseType_t stackWaterMark = uxTaskGetStackHighWaterMark(nullptr);
    //     //     if (stackWaterMark < 256) {
    //     //         logError("WARNING: Core 0 (Other Tasks) has had a LOW stack availablility: " + String(stackWaterMark));
    //     //     }
    //     // }
    // }

    unsigned long end_time = micros();
    tick_interval_sum += start_time - last_tick_time;
    tick_duration_sum += end_time - start_time;
    tick_interval_count++;
    last_tick_time = micros();
    if (millis() - last_log_time >= 10000) {  // Log every 10000ms (10 seconds)
        last_log_time = millis();
        unsigned long tick_interval = tick_interval_sum / tick_interval_count;
        avg_tick2_interval_us_ = tick_interval;
        avg_tick2_duration_us_ = tick_duration_sum / tick_interval_count;
        tick_duration_sum = 0;
        tick_interval_sum = 0;
        tick_interval_count = 0;
        // logDebug("Tick2 Avg: " + String(tick_interval) + "us (Core " + String(core_id) + ")");
    }
}

// LED Control Methods
bool LineRobotBoard::setLED(uint8_t led, bool on) {
    if (!validateLEDIndex(led) || !shift_register_) {
        return false;
    }
    shift_register_->set(led, on ? 1 : 0, false);
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
    std::lock_guard<std::mutex> lock(i2c_mutex_);
    if (!oled_ || line > 5) {
        return false;
    }
    oled_->setLine(line, text, centered);
    return true;
}

void LineRobotBoard::clearOLED() {
    std::lock_guard<std::mutex> lock(i2c_mutex_);
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
    return motor_left_ ? percentageFromMotorSpeed(motor_left_->getSpeed()) : 0;
}

int16_t LineRobotBoard::getMotorSpeedRight() const {
    std::lock_guard<std::mutex> lock(motor_mutex_);
    return motor_right_ ? percentageFromMotorSpeed(motor_right_->getSpeed()) : 0;
}

int LineRobotBoard::motorSpeedFromPercentage(int percentage) const {
    // Clamp percentage to -100 to 100
    if (percentage < -100) percentage = -100;
    if (percentage > 100) percentage = 100;

    // Scale the percentage to motor speed range
    float max_speed = (float)motor_max_voltage_ / ((float)motor_driver_input_voltage_ - (float)motor_driver_voltage_drop_) * (float)MAX_MOTOR_SPEED;
    int val = static_cast<int>(std::round(((float)percentage / 100.0) * max_speed));
    // logDebug("Max motor speed calculated as: " + String(max_speed) + ", Setting motor speed to: " + String(val) + " for percentage: " + String(percentage) + "%");
    // logger_->println("Converted percentage " + String(percentage) + "% to motor speed " + String(val) + " (Max Speed: " + String(max_speed) + ")");
    return val;
}

int LineRobotBoard::percentageFromMotorSpeed(int speed) const {
    // Scale the motor speed back to percentage
    float max_speed = (float)motor_max_voltage_ / ((float)motor_driver_input_voltage_ - (float)motor_driver_voltage_drop_) * (float)MAX_MOTOR_SPEED;
    
    // Clamp speed to -max_speed to max_speed
    if (speed < -max_speed) speed = -max_speed;
    if (speed > max_speed) speed = max_speed;

    return static_cast<int>(std::round(((float)speed / static_cast<float>(max_speed)) * 100.0));
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
        logError("Motor object not created");
        return false;
    }
    
    // Convert percentage speed to actual motor speed
    speed = motorSpeedFromPercentage(speed);

    // Check if speed is already set (avoid unnecessary operations)
    if (motor->getSpeed() == speed) {
        // logDebug("Motor speed already set to " + String(speed));
        return true;
    }
    
    int16_t speed_diff = abs(speed) - abs(motor->getSpeed());
    if (speed_diff > 30 && ramp_time_ms < 16) {
        // Force a ramp time of at least 16ms for large speed changes to avoid back EMS spikes froom the spinning motor
        ramp_time_ms = 16;
    }

    if (is_left) {
        target_motor_speed_left_ = speed;
        ramp_speed_diff_per_ms_left_ = (ramp_time_ms != 0) ?  std::ceil(  float(abs(speed_diff)) / float(ramp_time_ms)) : 0;
        // logDebug("Setting Left Motor Target Speed: " + String(speed) + " with ramp time: " + String(ramp_time_ms) + "ms, Diff/ms: " + String(ramp_speed_diff_per_ms_left_) + ", Motor Speed: " + String(motor->getSpeed()) + ", Speed Diff: " + String(speed_diff));
    } else {
        target_motor_speed_right_ = speed;
        ramp_speed_diff_per_ms_right_ = (ramp_time_ms != 0) ? std::ceil(  float(abs(speed_diff)) / float(ramp_time_ms)) : 0;
        // logDebug("Setting Right Motor Target Speed: " + String(speed) + " with ramp time: " + String(ramp_time_ms) + "ms, Diff/ms: " + String(ramp_speed_diff_per_ms_right_) + ", Motor Speed: " + String(motor->getSpeed()) + ", Speed Diff: " + String(speed_diff));
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
        //logDebug("Set " + String(is_left ? "Left" : "Right") + " Motor Speed: " + String(speed));
    }
    
    last_motor_update_ = millis();
    return true;
}

void LineRobotBoard::rampMotorSpeed(bool is_left, int16_t target_speed, int16_t current_speed, uint16_t ramp_speed_diff_per_ms) {
    if (target_speed == current_speed) {
        return;
    }
    if (ramp_speed_diff_per_ms == 0) {
        // No ramping needed - jumop to target speed
        if (is_left) {
            motor_left_->setSpeed(target_speed);
            if (state_) state_->motor_speed_left = target_speed;
        } else {
            motor_right_->setSpeed(target_speed);
            if (state_) state_->motor_speed_right = target_speed;
        }
        // logDebug("No ramping needed for " + String(is_left ? "Left" : "Right") + " Motor");
        return;
    }
    
    unsigned long now = millis();
    unsigned long elapsed_ms = last_motor_update_ > 0 ? now - last_motor_update_ : 1;
    if (elapsed_ms == 0) {
        elapsed_ms = 1;  // Ensure at least 1ms has elapsed
    }

    int16_t speed_change = static_cast<int16_t>(ramp_speed_diff_per_ms * elapsed_ms);
    if (target_speed < current_speed && speed_change > 0) {
        speed_change = -speed_change;  // Force Deceleration
    }
    int16_t new_speed = current_speed + speed_change;
    if ((ramp_speed_diff_per_ms > 0 && new_speed > target_speed) || 
        (ramp_speed_diff_per_ms < 0 && new_speed < target_speed)) {
        new_speed = target_speed;  // Clamp to target speed
    }

    if (is_left) {
        // logDebug("Ramping Left Motor Speed to " + String(new_speed) + ", Current Speed: " + String(current_speed) + ", Target Speed: " + String(target_speed) + ", Ramp Diff/ms: " + String(ramp_speed_diff_per_ms) + ", Elapsed ms: " + String(elapsed_ms));
        motor_left_->setSpeed(new_speed);
        if (state_) state_->motor_speed_left = new_speed;
    } else {
        // logDebug("Ramping Right Motor Speed to " + String(new_speed) + ", Current Speed: " + String(current_speed) + ", Target Speed: " + String(target_speed) + ", Ramp Diff/ms: " + String(ramp_speed_diff_per_ms) + ", Elapsed ms: " + String(elapsed_ms));
        motor_right_->setSpeed(new_speed);
        if (state_) state_->motor_speed_right = new_speed;
    }

    last_motor_update_ = now;
}

void LineRobotBoard::setShowIndicatorLEDsWhileRacing(bool show) {
    display_indicator_leds_while_racing_ = show;
}

void LineRobotBoard::tickInfraredSensors() {
    // Process all IR sensors
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    
    bool should_set_indicator_leds = shift_register_ != nullptr &&
                                     state_ != nullptr &&
                                     state_->currentState() != LINE_ROBOT_PRIMED && 
                                     !(state_->currentState() == LINE_ROBOT_RACING && !display_indicator_leds_while_racing_)
                                     ;

    for (uint8_t i = 0; i < NUM_IR_SENSORS; i++) {
        if (!ir_sensors_[i]) continue;
        
        ir_sensors_[i]->tick(true);  // Force ADC update
        
        // Update indicator LEDs (skip during PRIMED state to avoid distractions)
        if (should_set_indicator_leds) {
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
    
    // Protect I2C bus access with mutex to prevent conflicts
    std::lock_guard<std::mutex> lock(i2c_mutex_);
    
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
        if (state_->pose.x > 0.80f || state_->pose.x < -0.80f) {
            // Held Vertically, so reset baselines
            resetIRBaselines();
        } else {
            // Start baselining process
            baselineIRSensors();
        }
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

void LineRobotBoard::onBoardButton2PressedHandler(long time_pressed) {
    logDebug("Board Button2 Pressed: " + String(time_pressed) + "ms");
    if (on_board2_pressed_callback_ && on_board2_pressed_callback_(time_pressed)) {
        return;  // Custom handler processed the event
    }
    // No default action
}

void LineRobotBoard::onBoardButton2DoublePressedHandler(long time_between_presses) {
    logDebug("Board Button2 Double Pressed: " + String(time_between_presses) + "ms");
    if (on_board2_double_pressed_callback_ && 
        on_board2_double_pressed_callback_(time_between_presses)) {
        return;  // Custom handler processed the event
    }
    // No default action
}
void LineRobotBoard::onBoardButton2LongPressedHandler(long time_pressed) {
    logDebug("Board Button2 Long Pressed: " + String(time_pressed) + "ms");
    if (on_board2_long_pressed_callback_ && 
        on_board2_long_pressed_callback_(time_pressed)) {
        return;  // Custom handler processed the event
    }
    // No default action
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

void LineRobotBoard::onBoardButton2Pressed(std::function<bool(long)> callback) {
    on_board2_pressed_callback_ = callback;
}
void LineRobotBoard::onBoardButton2DoublePressed(std::function<bool(long)> callback) {
    on_board2_double_pressed_callback_ = callback;
}
void LineRobotBoard::onBoardButton2LongPressed(std::function<bool(long)> callback) {
    on_board2_long_pressed_callback_ = callback;
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
        HubHttpResponse response;
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
    
        HubHttpResponse resp;
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
    
        HubHttpResponse resp;
        resp.status = 200;
        resp.body = response_str.length() > 0 ? response_str : "No motor commands specified";
        resp.headers["Content-Type"] = "text/plain";
        return resp;
    });

    server_->on("/adc", [this](HttpRequest& req) {
        HubHttpResponse resp;
        resp.status = 200;
        resp.body.reserve(256);  // Pre-allocate to reduce fragmentation
        
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
        HubHttpResponse resp;
        resp.status = 200;
        resp.body.reserve(256);  // Pre-allocate to reduce fragmentation
        
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
            HubHttpResponse resp;
            resp.status = 500;
            resp.body = "State not available";
            resp.headers["Content-Type"] = "text/plain";
            return resp;
        }
        
        HubHttpResponse resp;
        resp.status = 200;
        resp.body.reserve(384);  // Pre-allocate to reduce fragmentation
        
        if (req.jsonRequested()) {
            resp.body = "{\n";
            resp.body += "  \"x\": " + String(state_->pose.x, 6) + ",\n";
            resp.body += "  \"y\": " + String(state_->pose.y, 6) + ",\n";
            resp.body += "  \"z\": " + String(state_->pose.z, 6) + ",\n";
            resp.body += "  \"speed\": " + String(state_->pose.speed, 6) + ",\n";
            resp.body += "  \"heading\": " + String(state_->pose.heading, 6) + ",\n";
            resp.body += "  \"last_update\": " + String(state_->pose.last_update_time) + "\n";
            resp.body += "}";
            
            resp.headers["Content-Type"] = "application/json";
            return resp;
        } else {
            resp.body = "Robot Pose & Motion:\n";
            resp.body += "  Acceleration X: " + String(state_->pose.x, 3) + " g\n";
            resp.body += "  Acceleration Y: " + String(state_->pose.y, 3) + " g\n";
            resp.body += "  Acceleration Z: " + String(state_->pose.z, 3) + " g\n";
            resp.body += "  Speed: " + String(state_->pose.speed, 3) + " m/s\n";
            resp.body += "  Heading: " + String(state_->pose.heading, 3) + " rad\n";
            resp.body += "  Last Update: " + String(state_->pose.last_update_time) + " ms\n";
        
            resp.headers["Content-Type"] = "text/plain";
            return resp;
        }
    });

    // Add new status and diagnostic routes
    server_->on("/status", [this](HttpRequest& req) {
        HubHttpResponse resp;
        resp.status = 200;
        resp.body = getSystemStatus();
        resp.headers["Content-Type"] = "application/json";
        return resp;
    });

    server_->on("/diagnostic", [this](HttpRequest& req) {
        HubHttpResponse resp;
        resp.status = 200;
        resp.body = performDiagnostic();
        resp.headers["Content-Type"] = "text/plain";
        return resp;
    });
    
    // Simple health check endpoint
    server_->on("/health", [this](HttpRequest& req) {
        HubHttpResponse resp;
        resp.status = 200;
        resp.body = "OK";
        resp.headers["Content-Type"] = "text/plain";
        return resp;
    });
    
    // Stack monitoring endpoint
    server_->on("/debug", [this](HttpRequest& req) {
        HubHttpResponse resp;
        resp.status = 200;
        
        UBaseType_t task0Stack = uxTaskGetStackHighWaterMark(nullptr);
        size_t freeHeap = ESP.getFreeHeap();
        size_t minFreeHeap = ESP.getMinFreeHeap();

        resp.body = "Tasks Min Free Stack:" + String(task0Stack) +
                   "\n            Free Heap:" + String(freeHeap) + 
                   "\n             Min Heap:" + String(minFreeHeap) +
                   "\n      Time since boot:" + String(getUptime()) + " ms";
        resp.headers["Content-Type"] = "text/plain";
        return resp;
    });

    server_->on("/baseline", [this](HttpRequest& req) {
        HubHttpResponse resp;
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
    return speed >= -100 && speed <= 100;
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

bool LineRobotBoard::addAlternateBoardButton2Pin(uint8_t pin, bool active_low) {
    if (alt_board_button2_) {
        logError("Alternate button2 already configured");
        return false;
    }
    
    try {
        alt_board_button2_ = std::unique_ptr<Button>(new Button(pin, 25, false, active_low, 1000, 500));
        alt_board_button2_->onPressed([this](long time) { onBoardButton2PressedHandler(time); });
        alt_board_button2_->onDoublePressed([this](long time) { onBoardButton2DoublePressedHandler(time); });
        alt_board_button2_->onLongPressed([this](long time) { onBoardButton2LongPressedHandler(time); });
        
        logDebug("Alternate board button2 configured on pin " + String(pin));
        return true;
    } catch (const std::exception& e) {
        logError("Failed to create alternate button2: " + String(e.what()));
        return false;
    }
}


// Diagnostic and Status Methods
bool LineRobotBoard::getAccelerometerReading(float& x, float& y, float& z) const {
    if (!lis3dh_) {
        return false;
    }
    
    // Protect I2C bus access with mutex
    std::lock_guard<std::mutex> lock(i2c_mutex_);
    
    x = lis3dh_->readFloatAccelX();
    y = lis3dh_->readFloatAccelY();
    z = lis3dh_->readFloatAccelZ();
    
    return true;
}

String LineRobotBoard::performDiagnostic() const {
    String report;
    report.reserve(1024);  // Pre-allocate to reduce fragmentation
    
    report = "=== LineRobot Diagnostic Report ===\n";
    
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
    String status;
    status.reserve(512);  // Pre-allocate to reduce fragmentation
    
    status = "{\n";
    status += "  \"version\": \"" + String(BOARD_VERSION) + "\",\n";
    status += "  \"type\": \"" + String(BOARD_TYPE) + "\",\n";
    status += "  \"uptime\": " + String(getUptime()) + ",\n";
    status += "  \"initialised\": " + String(is_initialised_ ? "true" : "false") + ",\n";
    status += "  \"debug\": " + String(debug_ ? "true" : "false") + ",\n";
    status += "  \"input_voltage\": " + String(voltageFromADCValue(last_voltage_adc_value_), 2) + ",\n";
    status += "  \"wifi_connected\": " + String(isWiFiConnected() ? "true" : "false") + ",\n";
    status += "  \"core_ticks\": " + String(getCoreTasksTickCount()) + ",\n";
    status += "  \"other_ticks\": " + String(getOtherTasksTickCount()) + ",\n";
    status += "  \"avg_tick1_interval_us\": " + String(avg_tick1_interval_us_) + ",\n";
    status += "  \"avg_tick2_interval_us\": " + String(avg_tick2_interval_us_) + ",\n";
    status += "  \"avg_tick1_duration_us\": " + String(avg_tick1_duration_us_) + ",\n";
    status += "  \"avg_tick2_duration_us\": " + String(avg_tick2_duration_us_) + ",\n";
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
bool LineRobotBoard::addHttpRoute(const String& route, std::function<void(HttpRequest*, HubHttpResponse*, LineRobotBoard*)> handler) {
    if (!server_) {
        logError("HTTP server not available");
        return false;
    }
    
    server_->on(route, [handler, this](HttpRequest& req) {
        HubHttpResponse response;
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
    pauseTimersAndTasks();
    
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    if (!initNVS(false)) {
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
    pauseTimersAndTasks();
    
    std::lock_guard<std::mutex> lock(sensor_mutex_);

    if (!initNVS(true)) {
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
        vTaskDelay(5);  // Small delay to avoid NVS issues
        
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

void LineRobotBoard::setInputVoltageAlertThreshold(float voltage) {
    input_voltage_alert_threshold_ = voltage;
    logDebug("Input voltage alert threshold set to " + String(voltage) + "v");
}

void LineRobotBoard::setMotorDriverVoltageDrop(uint16_t voltage_drop) {
    motor_driver_voltage_drop_ = voltage_drop;
    logDebug("Motor driver voltage drop set to " + String(voltage_drop) + "v");
}

void LineRobotBoard::setMotorDriverInputVoltage(uint16_t voltage) {
    motor_driver_input_voltage_ = voltage;
    logDebug("Motor driver input voltage set to " + String(voltage) + "v");
}

void LineRobotBoard::setMotorMaxVoltage(uint16_t voltage) {
    motor_max_voltage_ = voltage;
    logDebug("Motor max voltage set to " + String(voltage) + "v");
}

HubHttpClient* LineRobotBoard::getHttpClient() {
    return client_.get();
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
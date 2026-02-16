#ifndef LINEROBOT_BOARD_H
#define LINEROBOT_BOARD_H

#define BOARD_VERSION "2.3"
#define BOARD_TYPE "Hub Line Robot Board"

#include <Arduino.h>
#include <Preferences.h>
#include "linerobot_state.h"
#include <wifi_manager.h>
#include <http_server.h>
#include <http_client.h>
#include <hub_oled.h>
#include <serial_proxy.hpp>
#include <shift_register.h>
#include <dc_motor.h>
#include <LIS3DH/SparkFunLIS3DH.h>
#include <button.h>
#include <memory>
#include <array>
#include <mutex>
#include <atomic>

// Constants
#define INVALID_ANALOGUE_CHANNEL 65535
#define NUM_IR_SENSORS 4
#define MAX_ANALOG_VALUE 4095
#define MIN_MOTOR_SPEED -255
#define MAX_MOTOR_SPEED 255
#define DEFAULT_IR_THRESHOLD 400
#define BASELINE_SAMPLE_DURATION_MS 1000
#define BASELINE_SAMPLE_DELAY_MS 10
#define ADC_VOLTAGE_PIN 8   // ADC Pin attached to the voltage divider circuit

class InfraredSensor {
public:
    // Constructor with proper validation
    InfraredSensor(uint8_t analog_pin, uint8_t indicator_led_num, uint8_t ir_led_num, 
                   uint16_t threshold = DEFAULT_IR_THRESHOLD, int16_t baseline = 0);
    
    // Disable copy constructor and assignment operator for embedded systems
    InfraredSensor(const InfraredSensor&) = delete;
    InfraredSensor& operator=(const InfraredSensor&) = delete;
    
    // Move constructor and assignment are okay
    InfraredSensor(InfraredSensor&&) = default;
    InfraredSensor& operator=(InfraredSensor&&) = default;

    // Core functionality
    void tick(bool force_update = false);
    void setThreshold(uint16_t threshold);
    void setBaseline(int16_t baseline);
    void reset();

    // Getters with const correctness
    int16_t getValue() const { return value_; }
    uint16_t getRawValue() const { return raw_value_; }
    int16_t getBaseline() const { return baseline_; }
    uint16_t getThreshold() const { return threshold_; }
    uint8_t getAnalogPin() const { return analog_pin_; }
    uint8_t getIndicatorLedNum() const { return indicator_led_num_; }
    uint8_t getIrLedNum() const { return ir_led_num_; }
    bool isTriggered() const { return is_triggered_; }
    bool hasStateChanged() const { return state_changed_in_last_update_; }
    
    // Timing information
    unsigned long timeInLastState() const;
    unsigned long timeInCurrentState() const;

    // Utility functions
    String describe() const;
    String describeJson() const;

private:
    // Core sensor data
    int16_t value_;                    // Baseline-adjusted value
    uint16_t raw_value_;               // Raw ADC reading
    int16_t baseline_;                 // Baseline offset
    uint16_t threshold_;               // Trigger threshold
    
    // Hardware configuration
    uint8_t analog_pin_;               // ADC pin number
    uint8_t indicator_led_num_;        // LED indicator number
    uint8_t ir_led_num_;               // IR LED number
    
    // State tracking
    bool is_triggered_;                // Current trigger state
    bool state_changed_in_last_update_; // Flag for state change
    unsigned long time_state_changed_; // When state last changed
    unsigned long time_in_last_state_; // Duration of previous state
};

struct RobotConfig {
    int16_t ir_baselines[NUM_IR_SENSORS];
};


class LineRobotBoard {
    // Friend declarations for FreeRTOS task wrapper functions
    friend void coreTask0Wrapper(void* parameter);
    friend void coreTask1Wrapper(void* parameter);
    
public:
    // Disable copy constructor and assignment operator
    LineRobotBoard(const LineRobotBoard&) = delete;
    LineRobotBoard& operator=(const LineRobotBoard&) = delete;

private: 
    // Smart pointers for automatic memory management
    std::unique_ptr<WifiManager> wifi_man_;
    std::unique_ptr<HttpServer> server_;
    std::unique_ptr<HubHttpClient> client_;
    std::unique_ptr<HubOLED> oled_;
    std::unique_ptr<ShiftRegister> shift_register_;
    std::unique_ptr<DCMotor> motor_left_;
    std::unique_ptr<DCMotor> motor_right_;
    std::unique_ptr<LIS3DH> lis3dh_;
    std::unique_ptr<Button> board_button_;
    std::unique_ptr<Button> alt_board_button_;

    std::unique_ptr<Button> board_button2_;
    std::unique_ptr<Button> alt_board_button2_;
    
    
    // Use array for better memory management
    std::array<std::unique_ptr<InfraredSensor>, NUM_IR_SENSORS> ir_sensors_;
    
    // Raw pointers for externally managed resources
    CachingPrinter* logger_;           // Not owned by this class
    LineRobotState* state_;            // Not owned by this class
    
    // Configuration and state
    bool debug_;
    bool is_initialised_;
    bool timers_running_;
    bool use_internal_timers_;
    unsigned long start_millis_;
    RobotConfig config_;
    String ip_address_;
    unsigned long last_motor_update_;
    int target_motor_speed_left_;
    int target_motor_speed_right_;
    uint16_t ramp_speed_diff_per_ms_left_;
    uint16_t ramp_speed_diff_per_ms_right_;
    uint16_t last_voltage_adc_value_;
    float input_voltage_alert_threshold_;
    bool display_indicator_leds_while_racing_;

    float motor_driver_input_voltage_;
    float motor_driver_voltage_drop_;
    float motor_max_voltage_;
    
    // Atomic counters for thread safety
    std::atomic<unsigned long> tick1_count_{0};
    std::atomic<unsigned long> tick2_count_{0};

    // EEPROM/NVS management
    bool eeprom_valid_;
    Preferences preferences_;

    // Mutex for thread-safe operations
    mutable std::mutex sensor_mutex_;
    mutable std::mutex motor_mutex_;
    mutable std::mutex i2c_mutex_;

    // Callback functions
    std::function<bool(long)> on_board_pressed_callback_;
    std::function<bool(long)> on_board_double_pressed_callback_;
    std::function<bool(long)> on_board_long_pressed_callback_;
    std::function<bool(long)> on_board2_pressed_callback_;
    std::function<bool(long)> on_board2_double_pressed_callback_;
    std::function<bool(long)> on_board2_long_pressed_callback_;
    
    // Task handles for proper cleanup
    TaskHandle_t core_task0_handle_;
    TaskHandle_t core_task1_handle_;
    unsigned long avg_tick1_interval_us_;
    unsigned long avg_tick2_interval_us_;
    unsigned long avg_tick1_duration_us_;
    unsigned long avg_tick2_duration_us_;


    // Private helper methods
    void addDefaultRoutes();

    // Initialization helpers
    bool initShiftRegister();
    bool initMotors();
    bool initADC();
    bool initAccelerometer();
    bool initInfraredSensors(uint16_t ir_threshold);
    
    // Task management
    bool setupTimersAndTasks();
    void cleanupTimersAndTasks();
    void pauseTimersAndTasks();
    void resumeTimersAndTasks();
    
    // Core task functions
    void tick1();  // High-frequency sensor task (Core1, ~1kHz)
    void tick2();  // Lower-frequency misc tasks (Core0, ~62Hz)
    
    // Sensor processing
    void tickInfraredSensors();
    void tickAccelerometer();
    
    // Button event handlers
    void onBoardButtonPressedHandler(long time_pressed);
    void onBoardButtonDoublePressedHandler(long time_between_presses);
    void onBoardButtonLongPressedHandler(long time_pressed);
    void onBoardButton2PressedHandler(long time_pressed);
    void onBoardButton2DoublePressedHandler(long time_between_presses);
    void onBoardButton2LongPressedHandler(long time_pressed);
    
    // Baseline management
    void preBaseline();
    bool performBaselineCapture(uint8_t sensor_index);
    
    // Utility functions
    bool validateSensorIndex(uint8_t sensor) const;
    bool validateMotorSpeed(int16_t speed) const;
    bool validateLEDIndex(uint8_t led) const;
    void logDebug(const String& message) const;
    void logError(const String& message) const;
    
    // Thread-safe motor control helpers
    bool setMotorSpeedInternal(bool is_left, int16_t speed, uint16_t ramp_time_ms);
    void rampMotorSpeed(bool is_left, int16_t target_speed, int16_t current_speed, uint16_t ramp_speed_diff_per_ms);
    
    // Configuration management helpers
    String getConfigKey(const String& key) const;
    bool loadConfiguration();
    bool saveBaselines();
    bool loadBaselines();
    bool initNVS(bool read_only = true);
    bool closeNVS();
    
    float voltageFromADCValue(uint16_t adc_value) const;

    int motorSpeedFromPercentage(int percentage) const;
    int percentageFromMotorSpeed(int speed) const;
    
public:
        /**
         * @brief Construct a new Line Robot Board object
         * @param state The state object to use for the board (must not be null)
         * @note This object will be used to store the state of the robot and must be a subclass of LineRobotState
         * @param logger The logger to use for debugging (must not be null)
         * @param wifi_ssid The SSID of the WiFi network to connect to (optional)
         * @param wifi_password The password of the WiFi network to connect to (optional)
         * @param oled_type The type of OLED display to use (optional)
         * @param debug Whether to enable debug mode (optional)
         * @throws std::invalid_argument if state or logger is null
         */
        explicit LineRobotBoard(LineRobotState* state, CachingPrinter& logger, 
                               const String& wifi_ssid = "", 
                               const String& wifi_password = "", 
                               OledDisplayType oled_type = OLED_TYPE_NONE, 
                               bool debug = false, 
                               bool use_internal_timers = true);
        
        /**
         * @brief Destructor - properly cleans up all resources
         */
        ~LineRobotBoard();

        /**
         * @brief Initialize the board and start its internal tasks
         * @note This function will initialize the board and all of its components
         * @note This function will block until the initialization is complete
         * @param ir_threshold The threshold value for the IR sensors (default: 400)
         * @note The IR sensor threshold is the value that the (baseline adjusted) value of the IR sensor must be above to be considered triggered
         * @note You should call this function after setting up any HTTP routes you want to add to the server
         * @return true if initialization was successful, false otherwise
         */
        bool begin(uint16_t ir_threshold = DEFAULT_IR_THRESHOLD);
        
        /**
         * @brief Stop all timers and tasks, cleanup resources
         * @note Call this before destroying the object for proper cleanup
         */
        void end();
        
        /**
         * @brief Check if the board has been successfully initialized
         * @return true if initialized, false otherwise
         */
        bool isInitialized() const { return is_initialised_; }
       
        /**
         * @brief Set the debug mode for the board
         * @param debug Whether to enable or disable debug mode
         * @note In debug mode, additional logging will be printed to the logging printer (eg. Serial)
         */
        void setDebug(bool debug);
        
        /**
         * @brief Get current debug mode status
         * @return true if debug mode is enabled
         */
        bool isDebugEnabled() const { return debug_; }

        /**
         * @brief Get the tick count for the core robot tasks loop - this task runs on Core1 of the board (the second CPU core)
         * @note The tick count is wrapped every 60000 ticks (every 60 seconds)
         * @note This task targets running at ~1kHz (aka. every 1ms)
         * @return The current board tick count for the main robot task loop
         */
        unsigned long getCoreTasksTickCount() const;

        /**
         * @brief Get the tick count for the other tasks loop - this task runs on Core0 of the board (the first CPU core) and shares time with the ESP32 core tasks
         * @note The tick count is wrapped every 3600 ticks (every ~60 seconds)
         * @note This task targets running at ~62.5Hz (aka. every 16ms)
         * @note This task is shared with the ESP32 core tasks, so it may be blocked by other tasks
         * @note This task is also responsible for misc. tasks like processing requests to the HTTP server and observations of the board button state
         * @return The current board tick count for the other tasks loop
         */
        unsigned long getOtherTasksTickCount() const;
        
        /**
         * @brief Get uptime since board initialization
         * @return Uptime in milliseconds
         */
        unsigned long getUptime() const;

        /**
         * @brief Add a new HTTP route to the server
         * @param route The path route to add (e.g. "/my-route")
         * @note The route should start with a forward slash (e.g. "/my-route")
         * @param handler The function to call when the route is accessed
         * @note The handler function should take a HttpRequest, HttpResponse and LineRobotBoard as parameters
         * @return true if route was added successfully, false if server not available
         */
        bool addHttpRoute(const String& route, std::function<void(HttpRequest*, HubHttpResponse*, LineRobotBoard*)> handler);

        /**
         * @brief Set the state of the LED attached to the specified channel of the shift registers.
         * @param led The LED to set (0-15)
         * @param on Whether to turn the LED on or off
         * @return true if LED was set successfully, false if invalid LED number
         * @note This function is thread-safe
         */
        bool setLED(uint8_t led, bool on);
        
        /**
         * @brief Set multiple LEDs at once using a bitmask
         * @param mask 16-bit mask where each bit represents an LED state
         * @note This is more efficient than setting LEDs individually
         */
        void setLEDMask(uint16_t mask);
        
        /**
         * @brief Get the current state of all LEDs
         * @return 16-bit mask representing current LED states
         */
        uint16_t getLEDMask() const;

        /**
         * @brief Prints the specified text to the OLED display on the specified line
         * @param line The line to print to (0-5)
         * @param text The text to print (the text will be truncated to fit the line, the length of which will be determined by the OLED type)
         * @param centered Whether to center the text on the line (default: true)
         * @return true if text was set successfully, false if invalid line or OLED not available
         */
        bool setOLEDLine(uint8_t line, const String& text, bool centered = true);
        
        /**
         * @brief Clear the OLED display
         */
        void clearOLED();

        /**
         * @brief Set the percentage speed of the left motor with input validation and ramping
         * @param speed Motor speed (-100 to 100, negative values indicate reverse direction)
         * @param ramp_time_ms Time to ramp to new speed in milliseconds (0 = immediate)
         * @return true if speed was set successfully
         * @note This function is thread-safe
         */
        bool setMotorSpeedLeft(int16_t speed, uint16_t ramp_time_ms = 0);

        /**
         * @brief Set the percentage speed of the right motor with input validation and ramping
         * @param speed Motor speed (-100 to 100, negative values indicate reverse direction)  
         * @param ramp_time_ms Time to ramp to new speed in milliseconds (0 = immediate)
         * @return true if speed was set successfully
         * @note This function is thread-safe
         */
        bool setMotorSpeedRight(int16_t speed, uint16_t ramp_time_ms = 16);

        /**
         * @brief Set the percentage speed of both motors to the same value
         * @param speed Motor speed (-100 to 100, negative values indicate reverse direction)
         * @param ramp_time_ms Time to ramp to new speed in milliseconds (0 = immediate)
         * @return true if both speeds were set successfully
         */
        bool setMotorSpeedBoth(int16_t speed, uint16_t ramp_time_ms = 16);

        /**
         * @brief Set the percentage speed of the left and right motors independently
         * @param left Left motor speed (-100 to 100, negative values indicate reverse direction)
         * @param right Right motor speed (-100 to 100, negative values indicate reverse direction)
         * @param ramp_time_ms Time to ramp to new speed in milliseconds (0 = immediate)
         * @return true if both speeds were set successfully
         */
        bool setMotorSpeed(int16_t left, int16_t right, uint16_t ramp_time_ms = 16);
        
        /**
         * @brief Get current left motor percentage speed
         * @return Current left motor speed (-100 to 100)
         */
        int16_t getMotorSpeedLeft() const;
        
        /**
         * @brief Get current right motor percentage speed
         * @return Current right motor speed (-100 to 100)
         */
        int16_t getMotorSpeedRight() const;
        
        /**
         * @brief Emergency stop - immediately stop both motors
         */
        void emergencyStop();

        /**
         * @brief Reads the current analogue value (0-4095) from the specified GPIO pin
         * @param pin The GPIO pin to read from 
         * @return The current analogue value from the specified pin (0-4095)
         * @note This function will return INVALID_ANALOGUE_CHANNEL if the pin is invalid
         * @note For ESP32-S3, valid ADC pins are: 1-10, 11-20 (ADC1), 0, 2-9 (ADC2)
         */
        uint16_t readAnalogueValue(uint8_t pin) const;

        /**
         * @brief Set the baseline for all IR sensors automatically
         * @note This function will set the baseline for the IR sensors to the average of readings over 1 second
         * @note You must ensure that the robot is on the ground and NOT on the track when this function is called
         * @return true if baseline was set successfully
         */
        bool baselineIRSensors();
        
        /**
         * @brief Manually set the baseline for a specific IR sensor
         * @param sensor The sensor index (0-3)
         * @param baseline The baseline value to set
         * @return true if baseline was set successfully, false if invalid sensor
         */
        bool setIRSensorBaseline(uint8_t sensor, int16_t baseline);

        /**
         * @brief Set the threshold for a specific IR sensor
         * @param sensor The sensor to set the threshold for (0-3) [0=outer right, 1=inner right, 2=inner left, 3=outer left]
         * @param threshold The threshold to set (0-4095)
         * @return true if threshold was set successfully, false if invalid sensor
         */
        bool setIRSensorThreshold(uint8_t sensor, uint16_t threshold);
        
        /**
         * @brief Set the threshold for all IR sensors
         * @param threshold The threshold to set for all sensors (0-4095)
         */
        void setAllIRSensorThresholds(uint16_t threshold);
        
        /**
         * @brief Get the current raw value for the IR sensor, as last read from the ADC 
         * @param sensor The sensor to get the value from (0-3) [0=outer right, 1=inner right, 2=inner left, 3=outer left]
         * @return The current raw value of the IR sensor (0-4095), or INVALID_ANALOGUE_CHANNEL if invalid sensor
         */
        uint16_t getRawIRSensorValue(uint8_t sensor) const;
        
        /**
         * @brief Get the current baseline-adjusted value for the IR sensor
         * @param sensor The sensor to get the value from (0-3)
         * @return The baseline-adjusted value, or INT16_MAX if invalid sensor
         */
        int16_t getIRSensorValue(uint8_t sensor) const;
        
        /**
         * @brief Get the triggered state of an IR sensor
         * @param sensor The sensor to check (0-3)
         * @return true if sensor is triggered, false otherwise (or if invalid sensor)
         */
        bool isIRSensorTriggered(uint8_t sensor) const;
        
        /**
         * @brief Get a bitmask representing which IR sensors are triggered
         * @return 4-bit mask where bit 0=outer right, bit 1=inner right, bit 2=inner left, bit 3=outer left
         */
        uint8_t getIRSensorMask() const;

        /**
         * @brief Add an alternate pin to be used as a board button input in addition to the default board button pin (GPIO21)
         * @param pin The pin to use as an alternate board button input
         * @param active_low Whether the button is active low (default: true)
         * @return true if alternate button was added successfully
         */
        bool addAlternateBoardButtonPin(uint8_t pin, bool active_low = true);
        
        /**
         * @brief Add an alternate pin to be used as a board button 2 input in addition to the default board button pin (GPIO18)
         * @param pin The pin to use as an alternate board button input
         * @param active_low Whether the button is active low (default: true)
         * @return true if alternate button was added successfully
         */
        bool addAlternateBoardButton2Pin(uint8_t pin, bool active_low = true);



        /**
         * @brief Set the callback function to be called when the board button is pressed
         * @param callback The callback function to be called when the board button is pressed
         * @note The callback function will be called with the time in milliseconds that the button was held pressed down for
         * @note The callback function should return true if the event was handled, and false otherwise (and the default board handler will be called)
         */
        void onBoardButtonPressed(std::function<bool(long)> callback);
        
        /**
         * @brief Set the callback function to be called when the board button is double pressed
         * @param callback The callback function to be called when the board button is double pressed
         * @note The callback function will be called with the time in milliseconds between the two presses
         * @note The callback function should return true if the event was handled, and false otherwise (and the default board handler will be called)
         */
        void onBoardButtonDoublePressed(std::function<bool(long)> callback);
        
        /**
         * @brief Set the callback function to be called when the board button is long pressed
         * @param callback The callback function to be called when the board button is long pressed
         * @note The callback function will be called with the time in milliseconds that the button was held pressed down for
         * @note The callback function should return true if the event was handled, and false otherwise (and the default board handler will be called)
         */
        void onBoardButtonLongPressed(std::function<bool(long)> callback);

        /**
         * @brief Set the callback function to be called when the alternate board button (Button 2) is pressed
         * @param callback The callback function to be called when the alternate board button is pressed
         * @note The callback function will be called with the time in milliseconds that the button was held pressed down for
         * @note The callback function should return true if the event was handled, and false otherwise (and the default board handler will be called)
         */
        void onBoardButton2Pressed(std::function<bool(long)> callback);

        /**
         * @brief Set the callback function to be called when the alternate board button (Button 2) is double pressed
         * @param callback The callback function to be called when the alternate board button is double pressed
         * @note The callback function will be called with the time in milliseconds between the two presses
         * @note The callback function should return true if the event was handled, and false otherwise (and the default board handler will be called)
         */
        void onBoardButton2DoublePressed(std::function<bool(long)> callback);

        /**
         * @brief Set the callback function to be called when the alternate board button (Button 2) is long pressed
         * @param callback The callback function to be called when the alternate board button is long pressed
         * @note The callback function will be called with the time in milliseconds that the button was held pressed down for
         * @note The callback function should return true if the event was handled, and false otherwise (and the default board handler will be called)
         */
        void onBoardButton2LongPressed(std::function<bool(long)> callback);
        
        // New diagnostic and utility functions
        
        /**
         * @brief Get accelerometer readings
         * @param x Reference to store X acceleration (g-force)
         * @param y Reference to store Y acceleration (g-force)  
         * @param z Reference to store Z acceleration (g-force)
         * @return true if readings were successful
         */
        bool getAccelerometerReading(float& x, float& y, float& z) const;
        
        /**
         * @brief Perform a comprehensive system diagnostic
         * @return Diagnostic report as a string
         */
        String performDiagnostic() const;
        
        /**
         * @brief Get system status summary
         * @return JSON formatted status string
         */
        String getSystemStatus() const;
        
        /**
         * @brief Reset all IR sensor baselines to zero and clear saved values
         * @return true if reset was successful
         */
        bool resetIRBaselines();
        
        /**
         * @brief Save current configuration to non-volatile storage
         * @return true if save was successful
         */
        bool saveConfiguration();
        
        /**
         * @brief Enable or disable automatic IR sensor baseline correction
         * @param enable true to enable automatic correction
         * @note When enabled, baselines will be automatically adjusted during operation
         */
        void setAutoBaseline(bool enable);
        
        /**
         * @brief Check if WiFi is connected
         * @return true if connected to WiFi
         */
        bool isWiFiConnected() const;

        /**
         * @brief Get the current input voltage level, which is read from the board's voltage divider circuit. This is useful for monitoring the battery level.
         * @return Input voltage level in volts
         * @note Returns 0 if voltage reading is not available
         */
        float getInputVoltageLevel();

        /**
         * @brief Set whether to display indicator LEDs while racing (will slow things down slightly)
         */
        void setShowIndicatorLEDsWhileRacing(bool show);

        /**
         * @brief Get whether indicator LEDs are displayed while racing
         * @return true if indicator LEDs are displayed while racing
         */
        bool isShowingIndicatorLEDsWhileRacing() const { return display_indicator_leds_while_racing_; }

        /**
         * @brief Set the voltage threshold for low voltage alerts
         * @param threshold Voltage threshold in volts
         */
        void setInputVoltageAlertThreshold(float threshold);


        /**
         * Gets the HTTP client instance for making outbound requests.
         */
        HubHttpClient* getHttpClient();

        /**
         * @brief Set the motor driver input voltage to ennsure proper PWM scaling for the attached motors
         * @param voltage_mv Input voltage
         */
        void setMotorDriverInputVoltage(uint16_t voltage_mv);

        /**
         * @brief Set the motor driver voltage drop to ensure proper PWM scaling for the attached motors
         * @param voltage_drop_ Voltage drop across the motor driver
         */
        void setMotorDriverVoltageDrop(uint16_t voltage_drop);

        /**
         * @brief Set the maximum motor voltage to ensure proper PWM scaling for the attached motors
         * @param max_voltage Maximum motor voltage
         */
        void setMotorMaxVoltage(uint16_t max_voltage);
        
        /**
         * @brief Get the IP address assigned to the board (if connected to WiFi)
         * @return IP address as a string
         */
        String getIpAddress() const { return ip_address_; }


        /**
         * @brief Manually trigger a tick of the board's internal tasks
         * WARNING: Only use this if you have disabled the internal timers and tasks (when setting use_internal_timers to false on board construction)
         */
        void tick();
};

#endif  // LINEROBOT_BOARD_H
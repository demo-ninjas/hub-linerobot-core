#ifndef LINEROBOT_BOARD_H
#define LINEROBOT_BOARD_H

#define BOARD_VERSION "2.0"
#define BOARD_TYPE "ESP32"

#include <Arduino.h>
#include <Preferences.h>
#include "linerobot_state.h"
#include <wifi_manager.h>
#include <http_server.h>
#include <hub_oled.h>
#include <serial_proxy.hpp>
#include <shift_register.h>
#include <dc_motor.h>
#include <LIS3DH/SparkFunLIS3DH.h>
#include <button.h>


#define INVALID_ANALOGUE_CHANNEL 0xFFFFFFFF

static unsigned long start_time = millis();

class InfraredSensor {
    public: 
        int value;
        int raw_value;
        int baseline;
        int threshold;
        int indicator_led_num;
        int ir_led_num;
        int analog_pin;
        bool is_triggered;
        bool state_changed_in_last_update;
        unsigned long time_state_changed;
        unsigned long time_in_last_state;


        InfraredSensor(int analog_pin, int indicator_led_num, int ir_led_num, int threshold = 0, int baseline = 0);
        void tick(bool channelRequiredUpdate = false);
        void setThreshold(int threshold);
        void setBaseline(int baseline);

        unsigned long timeInLastState();
        unsigned long timeInCurrentState();

        String describe();
        String describeJson();
};

class LineRobotBoard {
    private: 
        WifiManager* wifi_man;
        HttpServer* server;
        HubOLED* oled;
        CachingPrinter* logger;
        LineRobotState* state;
        bool debug;
        unsigned long start_millis;
        unsigned long tick1_count;
        unsigned long tick2_count;

        bool eeprom_valid;
        Preferences preferences;

        ShiftRegister* shift_register;
        DCMotor* motor_left;
        DCMotor* motor_right;
        InfraredSensor* ir_sensors[4];
        LIS3DH* lis3dh;
        Button* board_button;

        std::function<bool(long)> onBoardPressedCallback;
        std::function<bool(long)> onBoardDoublePressedCallback;
        std::function<bool(long)> onBoardLongPressedCallback;


        void addDefaultRoutes();

        void initShiftRegister();
        void initMotors();
        void initADC();
        void initAccelerometer();
        void initInfraredSensors(int ir_threshold);

        void tickInfraredSensors();
        void tickAccelerometer();

        void onBoardButtonPressedHandler(long timePressed);
        void onBoardButtonDoublePressedHandler(long timeBetweenPresses);
        void onBoardButtonLongPressedHandler(long timePressed);

        void preBaseline();

        void setupTimersAndTasks();
        void pauseTimersAndTasks();
        void resumeTimersAndTasks();

         /**
         * @brief Tick the main sensor task - on the second CPU core (Core1) - it assumes that the ESP32 core tasks are running on the first core and will not block them
         */
        void tick1();

        /**
         * @brief Tick the misc. tasks - on the first CPU core (Core0) - shared with the ESP32 core tasks
         */
        void tick2();
    public:         
        /**
         * @brief Construct a new Line Robot Board object
         * @param state The state object to use for the board
         * @note This object will be used to store the state of the robot and must be a subclass of LineRobotState
         * @param logger The logger to use for debugging
         * @param wifi_ssid The SSID of the WiFi network to connect to (optional)
         * @param wifi_password The password of the WiFi network to connect to (optional)
         * @param oledType The type of OLED display to use (optional)
         * @param debug Whether to enable debug mode (optional)
         */
        LineRobotBoard(LineRobotState* state, CachingPrinter& logger, String wifi_ssid = "", String wifi_password = "", OledDisplayType oledType = OLED_TYPE_NONE, bool debug = false);
        ~LineRobotBoard();

        /**
         * @brief Initialize the board and start its internal tasks
         * @note This function will initialize the board and all of its components
         * @note This function will block until the initialization is complete
         * @param ir_threshold The threshold value for the IR sensors (default: 400)
         * @note The IR sensor threshold is the value that the (baseline adjusted) value of the IR sensor must be above to be considered triggered
         * @note You should call this function after setting up any HTTP routes you want to add to the server
         */
        void begin(int ir_threshold = 400); // Default IR threshold is 400
       
        /**
         * @brief Set the debug mode for the board
         * @param debug Whether to enable or disable debug mode
         * @note In debug mode, additional logging will be printed to the logging printer (eg. Serial)
         */
        void setDebug(bool debug);

        /**
         * @brief Get the tick count for the core robot tasks loop - this task runs on Core1 of the board (the second CPU core)
         * @note This task targets running at ~500Hz (aka. every 2ms)
         * @return The current board tick count for the main robot task loop
         */
        unsigned long getCoreTasksTickCount();

        /**
         * @brief Get the tick count for the other tasks loop - this task runs on Core0 of the board (the first CPU core) and shares time with the ESP32 core tasks
         * @note This task targets running at ~31.25Hz (aka. every 32ms)
         * @note This task is shared with the ESP32 core tasks, so it may be blocked by other tasks
         * @note This task is also responsible for misc. tasks like processing requests to the HTTP server and observations of the board button state
         * @return The current board tick count for the other tasks loop
         */
        unsigned long getOtherTasksTickCount();

        /**
         * @brief Add a new HTTP route to the server
         * @param route The path route to add (e.g. "/my-route")
         * @note The route should start with a forward slash (e.g. "/my-route")
         * @param handler The function to call when the route is accessed
         * @note The handler function should take a HttpRequest, HttpResponse and LineRobotBoard as parameters
         */
        void addHttpRoute(String route, std::function<void(HttpRequest*, HttpResponse*, LineRobotBoard*)> handler);

        /**
         * @brief Set the state of the LED attached to the specified channel of the shift registers.
         * @param led The LED to set (0-15)
         * @param on Whether to turn the LED on or off
         * @note This function will block until the LED is set
         */
        void setLED(int led, bool on);

        /**
         * @brief Prints  the specified text to the OLED display on the specified line
         * @param line The line to print to (0-5)
         * @param text The text to print (the text will be truncated to fit the line, the length of which will be determined by the OLED type)
         * @param centered Whether to center the text on the line (default: true)
         * @note This function will block until the text is printed
         */
        void setOLEDLine(int line, String text, bool centered = true);

        /**
         * @brief Set the speed of the left motor (-255-255)
         * @note The speed is a value between -255 and 255, where negative values indicate reverse direction
         */
        void setMotorSpeedLeft(int speed);

        /**
         * @brief Set the speed of the right motor (-255-255)
         * @note The speed is a value between -255 and 255, where negative values indicate reverse direction
         */
        void setMotorSpeedRight(int speed);

        /**
         * @brief Set the speed of both motors (-255-255) to be the same
         * @note The speed is a value between -255 and 255, where negative values indicate reverse direction
         * @param speed The speed of both motors (-255-255)
         */
        void setMotorSpeedBoth(int speed);

        /**
         * @brief Set the speed of the left and right motors (-255-255)
         * @note The speed is a value between -255 and 255, where negative values indicate reverse direction
         * @param left The speed of the left motor (-255-255)
         * @param right The speed of the right motor (-255-255)
         */
        void setMotorSpeed(int left, int right);

        /**
         * @brief Reads the current analogue value (0-4095) from the specified channel
         * @param channel The channel to read from (0-7)
         * @return The current analogue value from the specified channel (0-4095)
         * @note This function will block until the value is read
         * @note This function will return INVALID_ANALOGUE_CHANNEL if the channel is invalid
         */
        int readAnalogueValue(int channel);

        /**
         * @brief Set the baseline for the IR sensors
         * @note This function will set the baseline for the IR sensors to the average of the last ~100 readings
         * @note This function will block until the baseline is set, and you must ensure that the robot is on the ground and NOT on the track when this function is called
         */
        void baselineIRSensors();

        /**
         * @brief Set the threshold for the IR sensors, the (baseline adjusted) value is the value that the IR sensor must be above to be considered triggered
         * @param sensor The sensor to set the threshold for (0-3) [0 == outer right, 1 == inner right, 2 == inner left, 3 == outer left]
         * @param threshold The threshold to set (0-4095)
         * @note This function will set the threshold for the IR sensors to the specified value
         * @note This function will return INVALID_ANALOGUE_CHANNEL if you provide an invalid sensor id
         */
        void setIRSensorThreshold(int sensor, int threshold);
        
        /**
         * @brief Get the current raw value of for the IR sensor, as last read from the ADC 
         * @param sensor The sensor to get the value from (0-3) [0 == outer right, 1 == inner right, 2 == inner left, 3 == outer left]
         * @return The current value of the IR sensor will be in the range (0-4095)
         */
        int getRawIRSensorValue(int sensor);


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

};

#endif  // LINEROBOT_BOARD_H
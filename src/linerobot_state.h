#ifndef LINEROBOT_STATE_H
#define LINEROBOT_STATE_H

#include <Arduino.h>

class LineRobotPose {
    public: 
        float x;
        float y;
        float z;
        float speed; // m/s
        float heading; // radians
        unsigned long last_update_time;
        
        void updatePose(float x, float y, float z) {
            unsigned long now = millis();
            float deltaTime = (now - this->last_update_time) / 1000.0; // Convert milliseconds to seconds
            float deltaX = x - this->x;
            float deltaY = y - this->y;
            float deltaZ = z - this->z;
            float distance = sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
            this->speed = distance / deltaTime;
            this->heading = atan2(deltaY, deltaX);
            this->last_update_time = now;
            this->x = x;
            this->y = y;
            this->z = z;
        }
};


#define LINE_ROBOT_IDLE 0
#define LINE_ROBOT_PRIMED 1 
#define LINE_ROBOT_RACING 2
#define LINE_ROBOT_BASELINE_INIT 20
#define LINE_ROBOT_BASELINING 21

class LineRobotState {
    public: 
        
        /**
         * @brief The pose of the robot
         * @note You can use this to get the current pose of the robot
         * @note The pose is a LineRobotPose object that contains the x, y, z coordinates of the robot, the acceleration speed of the robot in m/s, and the heading of the robot in radians
         */
        LineRobotPose pose;

        /**
         * @brief The speed of the Left DC Motor on the robot board
         * @note This value is the speed of the motor in the range of -255 to 255, where negative values indicate reverse direction
         */
        int motor_speed_left;

        /**
         * @brief The speed of the Right DC Motor on the robot board
         * @note This value is the speed of the motor in the range of -255 to 255, where negative values indicate reverse direction
         */
        int motor_speed_right;

        /**
         * @brief The value of the Leftmost IR Sensor on the robot board
         * @note This value is the baseline adjusted value of the IR sensor
         */
        int ir_outer_left;
        /**
         * @brief A boolean indicating if the Leftmost IR sensor is triggered (aka. if the value is greater than the threshold)
         * @note This value is true if the IR sensor is triggered, and false otherwise
         */
        bool ir_outer_left_triggered;
        /**
         * @brief The value of the Left middle IR sensor on the robot board
         * @note This value is the baseline adjusted value of the IR sensor
         */
        int ir_inner_left;
        /**
         * @brief A boolean indicating if the Left middle IR sensor is triggered (aka. if the value is greater than the threshold)
         * @note This value is true if the IR sensor is triggered, and false otherwise
         */
        bool ir_inner_left_triggered;
        /**
         * @brief The value of the Right middle IR sensor on the robot board
         * @note This value is the baseline adjusted value of the IR sensor
         */
        int ir_inner_right;
        /**
         * @brief A boolean indicating if the Right middle IR sensor is triggered (aka. if the value is greater than the threshold)
         * @note This value is true if the IR sensor is triggered, and false otherwise
         */
        bool ir_inner_right_triggered;
        /**
         * @brief The value of the Rightmost IR Sensor on the robot board
         * @note This value is the baseline adjusted value of the IR sensor
         */
        int ir_outer_right;
        /**
         * @brief A boolean indicating if the Rightmost IR sensor is triggered (aka. if the value is greater than the threshold)
         * @note This value is true if the IR sensor is triggered, and false otherwise
         */
        bool ir_outer_right_triggered;
        
        /**
         * @brief Construct a new LineRobotState object
         * @note This object will be used to store the state of the robot and must be a subclass of LineRobotState
         * @note The "state" is a value between 0 and 255, where 0 = IDLE, 1 = PRIMED, 2 = RACING, 20 = BASELINE_INIT, 21 = BASELINING, and any other value is an unknown state that you can define for your own purposes
         */
        LineRobotState();

        /**
         * @brief Get the current state of the robot
         * @return The current state of the robot
         */
        uint8_t currentState();

        /**
         * @brief Get the previous state of the robot, aka. the state that the robot was in before it's current state
         * @return The previous state of the robot
         */
        uint8_t previousState();

        /**
         * @brief Set the state of the robot
         * @param state The state to set the robot to
         * @note The state is a value between 0 and 255, where 0 = IDLE, 1 = PRIMED, 2 = RACING, 20 = BASELINE_INIT, 21 = BASELINING, and any other value is an unknown state that you can define for your own purposes
         * @note This function will call the appropriate start and end callbacks for the state
         * @note This function will also call the onStateChange callback if it is set
         */
        void setState(uint8_t state);

        /**
         * @brief Set the callback function to be called when the IDLE state is entered
         */
        void onIDLEStart(std::function<void()> callback);
        /**
         * @brief Set the callback function to be called when the IDLE state is exited
         */
        void onIDLEEnd(std::function<void()> callback);
        /**
         * @brief Set the callback function to be called when the PRIMED state is entered
         */
        void onPRIMEDStart(std::function<void()> callback);
        /**
         * @brief Set the callback function to be called when the PRIMED state is exited
         */
        void onPRIMEDEnd(std::function<void()> callback);
        /**
         * @brief Set the callback function to be called when the RACING state is entered
         */
        void onRACINGStart(std::function<void()> callback);
        /**
         * @brief Set the callback function to be called when the RACING state is exited
         */
        void onRACINGEnd(std::function<void()> callback);
        /**
         * @brief Set the callback function to be called when the BASELINE_INIT state is entered
         */
        void onBASELINEINITStart(std::function<void()> callback);
        /**
         * @brief Set the callback function to be called when the BASELINE_INIT state is exited
         */
        void onBASELINEINITEnd(std::function<void()> callback);
        /**
         * @brief Set the callback function to be called when the BASELINING state is entered
         */
        void onBASELININGStart(std::function<void()> callback);
        /**
         * @brief Set the callback function to be called when the BASELINING state is exited
         */
        void onBASELININGEnd(std::function<void()> callback);
        /**
         * @brief Set the callback function to be called when the state changes
         * @param callback The callback function to be called when the state changes
         * @note The callback function will be called with the new state as the first parameter and the old state as the second parameter
         */
        void onStateChange(std::function<void(uint8_t, uint8_t)> callback);

    private: 
        uint8_t state; // 0 = idle, 1 = primed, 2 = racing, 3 = baseline start, 4 = baselining, etc...
        uint8_t prevState; // The previous state of the robot
        std::function<void(uint8_t, uint8_t)> onStateChangeCallback;
        std::function<void()> onIDLEStartCallback;
        std::function<void()> onIDLEEndCallback;
        std::function<void()> onPRIMEDStartCallback;
        std::function<void()> onPRIMEDEndCallback;
        std::function<void()> onRACINGStartCallback;
        std::function<void()> onRACINGEndCallback;
        std::function<void()> onBASELINEINITStartCallback;
        std::function<void()> onBASELINEINITEndCallback;
        std::function<void()> onBASELININGStartCallback;
        std::function<void()> onBASELININGEndCallback;
};

#endif  // LINEROBOT_STATE_H

#include "linerobot_state.h"

LineRobotState::LineRobotState() {
    this->state = LINE_ROBOT_IDLE;
    this->prevState = LINE_ROBOT_IDLE;
    this->onIDLEStartCallback = nullptr;
    this->onIDLEEndCallback = nullptr;
    this->onPRIMEDStartCallback = nullptr;
    this->onPRIMEDEndCallback = nullptr;
    this->onRACINGStartCallback = nullptr;
    this->onRACINGEndCallback = nullptr;
    this->onBASELINEINITStartCallback = nullptr;
    this->onBASELINEINITEndCallback = nullptr;
    this->onBASELININGStartCallback = nullptr;
    this->onBASELININGEndCallback = nullptr;
    this->onStateChangeCallback = nullptr;

    this->pose.x = 0.0f;
    this->pose.y = 0.0f;
    this->pose.z = 0.0f;
    this->pose.speed = 0.0f;
    this->pose.heading = 0.0f;
    this->pose.last_update_time = 0;
    this->motor_speed_left = 0;
    this->motor_speed_right = 0;
    this->ir_outer_left = 0;
    this->ir_inner_left = 0;
    this->ir_inner_right = 0;
    this->ir_outer_right = 0;
    this->ir_outer_left = 0;
    this->ir_inner_left_triggered = false;
    this->ir_inner_right_triggered = false;
    this->ir_outer_left_triggered = false;
    this->ir_outer_right_triggered = false;
    
}

void LineRobotState::setState(uint8_t state) {
    if (this->state != state) {
        // Call the end callback for the previous state (if it's a KNOWN state)
        switch (this->state) {
            case LINE_ROBOT_IDLE:
                if (this->onIDLEEndCallback) {
                    this->onIDLEEndCallback();
                }
                break;
            case LINE_ROBOT_PRIMED:
                if (this->onPRIMEDEndCallback) {
                    this->onPRIMEDEndCallback();
                }
                break;
            case LINE_ROBOT_RACING:
                if (this->onRACINGEndCallback) {
                    this->onRACINGEndCallback();
                }
                break;
            case LINE_ROBOT_BASELINE_INIT:
                if (this->onBASELINEINITEndCallback) {
                    this->onBASELINEINITEndCallback();
                }
                break;
            case LINE_ROBOT_BASELINING:
                if (this->onBASELININGEndCallback) {
                    this->onBASELININGEndCallback();
                }
                break;
        }

        this->prevState = this->state; // Store the previous state
        this->state = state;
        if (this->onStateChangeCallback) {
            this->onStateChangeCallback(state, this->prevState);
        }

        // Call the start callback for the new state (if it's a KNOWN state)
        switch (state) {
            case LINE_ROBOT_IDLE:
                if (this->onIDLEStartCallback) {
                    this->onIDLEStartCallback();
                }
                break;
            case LINE_ROBOT_PRIMED:
                if (this->onPRIMEDStartCallback) {
                    this->onPRIMEDStartCallback();
                }
                break;
            case LINE_ROBOT_RACING:
                if (this->onRACINGStartCallback) {
                    this->onRACINGStartCallback();
                }
                break;
            case LINE_ROBOT_BASELINE_INIT:
                if (this->onBASELINEINITStartCallback) {
                    this->onBASELINEINITStartCallback();
                }
                break;
            case LINE_ROBOT_BASELINING:
                if (this->onBASELININGStartCallback) {
                    this->onBASELININGStartCallback();
                }
                break;
        }
    }
}

void LineRobotState::onIDLEStart(std::function<void()> callback) {
    this->onIDLEStartCallback = callback;
}
void LineRobotState::onIDLEEnd(std::function<void()> callback) {
    this->onIDLEEndCallback = callback;
}

void LineRobotState::onPRIMEDStart(std::function<void()> callback) {
    this->onPRIMEDStartCallback = callback;
}
void LineRobotState::onPRIMEDEnd(std::function<void()> callback) {
    this->onPRIMEDEndCallback = callback;
}

void LineRobotState::onRACINGStart(std::function<void()> callback) {
    this->onRACINGStartCallback = callback;
}
void LineRobotState::onRACINGEnd(std::function<void()> callback) {
    this->onRACINGEndCallback = callback;
}

void LineRobotState::onBASELINEINITStart(std::function<void()> callback) {
    this->onBASELINEINITStartCallback = callback;
}
void LineRobotState::onBASELINEINITEnd(std::function<void()> callback) {
    this->onBASELINEINITEndCallback = callback;
}

void LineRobotState::onBASELININGStart(std::function<void()> callback) {
    this->onBASELININGStartCallback = callback;
}
void LineRobotState::onBASELININGEnd(std::function<void()> callback) {
    this->onBASELININGEndCallback = callback;
}

void LineRobotState::onStateChange(std::function<void(uint8_t, uint8_t)> callback) {
    this->onStateChangeCallback = callback;
}

uint8_t LineRobotState::currentState() {
    return this->state;
}

uint8_t LineRobotState::previousState() {
    return this->prevState;
}

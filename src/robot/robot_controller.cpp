#include "robot_controller.h"
#include <cmath>

RobotController::RobotController(double max_linear_velocity, double max_angular_velocity)
    : max_linear_velocity_(max_linear_velocity), max_angular_velocity_(max_angular_velocity),
      current_linear_velocity_(0.0), current_angular_velocity_(0.0),
      current_command_(ControlCommand::STOP), x_(0.0), y_(0.0), theta_(0.0) {
    last_update_ = std::chrono::steady_clock::now();
    last_command_time_ = std::chrono::steady_clock::now();
}

void RobotController::setControlCommand(ControlCommand command) {
    current_command_ = command;
    last_command_time_ = std::chrono::steady_clock::now();
}

RobotState RobotController::updateState() {
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update_).count() / 1000.0;
    
    if (dt <= 0) dt = 0.001; // Minimum time delta
    
    updateVelocities();
    integrateMotion(dt);
    
    RobotState state;
    // Position is estimated by GTSAM, not the controller
    state.x = 0.0;  // Will be overridden by GTSAM estimate
    state.y = 0.0;  // Will be overridden by GTSAM estimate  
    state.theta = 0.0;  // Will be overridden by GTSAM estimate
    state.linear_velocity = current_linear_velocity_;
    state.angular_velocity = current_angular_velocity_;
    state.timestamp = now;
    
    last_update_ = now;
    return state;
}

void RobotController::updateVelocities() {
    ControlCommand command = current_command_.load();
    
    // Check if we should auto-stop due to timeout (for safety)
    if (shouldAutoStop()) {
        current_command_ = ControlCommand::STOP;
        command = ControlCommand::STOP;
    }
    
    // Robot only moves while command is actively held
    // When command is released, it immediately stops
    switch (command) {
        case ControlCommand::FORWARD:
            current_linear_velocity_ = max_linear_velocity_;
            current_angular_velocity_ = 0.0;
            break;
        case ControlCommand::BACKWARD:
            current_linear_velocity_ = -max_linear_velocity_;
            current_angular_velocity_ = 0.0;
            break;
        case ControlCommand::LEFT:
            current_linear_velocity_ = 0.0;
            current_angular_velocity_ = max_angular_velocity_;
            break;
        case ControlCommand::RIGHT:
            current_linear_velocity_ = 0.0;
            current_angular_velocity_ = -max_angular_velocity_;
            break;
        case ControlCommand::STOP:
        default:
            // Immediate stop when no command or STOP command
            current_linear_velocity_ = 0.0;
            current_angular_velocity_ = 0.0;
            break;
    }
}

void RobotController::integrateMotion(double dt) {
    // Don't integrate position here - let GTSAM handle the position estimation
    // This controller only provides velocity commands
    // The actual position will be estimated by GTSAM based on sensor data
}

void RobotController::reset() {
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;
    current_linear_velocity_ = 0.0;
    current_angular_velocity_ = 0.0;
    current_command_ = ControlCommand::STOP;
    last_update_ = std::chrono::steady_clock::now();
    last_command_time_ = std::chrono::steady_clock::now();
}

bool RobotController::shouldAutoStop() {
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    long time_since_last_command = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_command_time_).count();
    
    // Auto-stop if no command received for more than the timeout period
    // and the current command is not STOP
    return (time_since_last_command > AUTO_STOP_TIMEOUT_MS) && (current_command_.load() != ControlCommand::STOP);
}

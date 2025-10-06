#include "robot_controller.h"
#include <cmath>

RobotController::RobotController(double max_linear_velocity, double max_angular_velocity, double acceleration)
    : max_linear_velocity_(max_linear_velocity), max_angular_velocity_(max_angular_velocity),
      acceleration_(acceleration), current_linear_velocity_(0.0), current_angular_velocity_(0.0),
      target_linear_velocity_(0.0), target_angular_velocity_(0.0),
      current_command_(ControlCommand::STOP), x_(0.0), y_(0.0), theta_(0.0) {
    last_update_ = std::chrono::steady_clock::now();
    last_command_time_ = std::chrono::steady_clock::now();
}

void RobotController::setControlCommand(ControlCommand command) {
    current_command_ = command;
    last_command_time_ = std::chrono::steady_clock::now();
}

void RobotController::setMaxSpeed(double max_linear_velocity, double max_angular_velocity) {
    max_linear_velocity_ = max_linear_velocity;
    max_angular_velocity_ = max_angular_velocity;
}

void RobotController::setAcceleration(double acceleration) {
    acceleration_ = acceleration;
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
    
    // Set target velocities based on command
    switch (command) {
        case ControlCommand::FORWARD:
            target_linear_velocity_ = max_linear_velocity_;
            target_angular_velocity_ = 0.0;
            break;
        case ControlCommand::BACKWARD:
            target_linear_velocity_ = -max_linear_velocity_;
            target_angular_velocity_ = 0.0;
            break;
        case ControlCommand::LEFT:
            target_linear_velocity_ = 0.0;
            target_angular_velocity_ = max_angular_velocity_;
            break;
        case ControlCommand::RIGHT:
            target_linear_velocity_ = 0.0;
            target_angular_velocity_ = -max_angular_velocity_;
            break;
        case ControlCommand::STOP:
        default:
            // Target stop when no command or STOP command
            target_linear_velocity_ = 0.0;
            target_angular_velocity_ = 0.0;
            break;
    }
    
    // Apply acceleration/deceleration to reach target velocities
    double dt = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - last_update_).count() / 1000.0;
    
    if (dt <= 0) dt = 0.001; // Minimum time delta
    
    // Linear velocity acceleration
    double linear_diff = target_linear_velocity_ - current_linear_velocity_;
    double linear_accel = acceleration_ * dt;
    
    if (std::abs(linear_diff) <= linear_accel) {
        current_linear_velocity_ = target_linear_velocity_;
    } else {
        current_linear_velocity_ += (linear_diff > 0 ? linear_accel : -linear_accel);
    }
    
    // Angular velocity acceleration
    double angular_diff = target_angular_velocity_ - current_angular_velocity_;
    double angular_accel = acceleration_ * dt;
    
    if (std::abs(angular_diff) <= angular_accel) {
        current_angular_velocity_ = target_angular_velocity_;
    } else {
        current_angular_velocity_ += (angular_diff > 0 ? angular_accel : -angular_accel);
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
    target_linear_velocity_ = 0.0;
    target_angular_velocity_ = 0.0;
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

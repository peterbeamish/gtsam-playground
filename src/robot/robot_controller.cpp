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
    
    // Check if we should auto-stop due to timeout
    if (shouldAutoStop()) {
        current_command_ = ControlCommand::STOP;
    }
    
    updateVelocities();
    integrateMotion(dt);
    
    RobotState state;
    state.x = x_;
    state.y = y_;
    state.theta = theta_;
    state.linear_velocity = current_linear_velocity_;
    state.angular_velocity = current_angular_velocity_;
    state.timestamp = now;
    
    last_update_ = now;
    return state;
}

void RobotController::updateVelocities() {
    ControlCommand command = current_command_.load();
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
            current_linear_velocity_ = 0.0;
            current_angular_velocity_ = 0.0;
            break;
    }
}

void RobotController::integrateMotion(double dt) {
    // Simple kinematic integration
    double delta_x = current_linear_velocity_ * std::cos(theta_) * dt;
    double delta_y = current_linear_velocity_ * std::sin(theta_) * dt;
    double delta_theta = current_angular_velocity_ * dt;
    
    x_ += delta_x;
    y_ += delta_y;
    theta_ += delta_theta;
    
    // Normalize angle
    while (theta_ > M_PI) theta_ -= 2.0 * M_PI;
    while (theta_ < -M_PI) theta_ += 2.0 * M_PI;
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

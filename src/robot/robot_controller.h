#pragma once

#include <chrono>
#include <atomic>

struct RobotState {
    double x;
    double y;
    double theta;
    double linear_velocity;
    double angular_velocity;
    std::chrono::steady_clock::time_point timestamp;
};

enum ControlCommand {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    STOP
};

class RobotController {
public:
    RobotController(double max_linear_velocity = 1.0, double max_angular_velocity = 1.0);
    
    void setControlCommand(ControlCommand command);
    RobotState updateState();
    void reset();
    
private:
    double max_linear_velocity_;
    double max_angular_velocity_;
    double current_linear_velocity_;
    double current_angular_velocity_;
    
    std::atomic<ControlCommand> current_command_;
    std::chrono::steady_clock::time_point last_update_;
    std::chrono::steady_clock::time_point last_command_time_;
    
    // Robot state
    double x_;
    double y_;
    double theta_;
    
    // Auto-stop timeout (in milliseconds)
    static const int AUTO_STOP_TIMEOUT_MS = 200;
    
    void updateVelocities();
    void integrateMotion(double dt);
    bool shouldAutoStop();
};

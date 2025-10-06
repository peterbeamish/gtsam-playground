#pragma once

#include <chrono>
#include <random>
#include <cmath>

struct WheelEncoderData {
    double left_wheel_velocity;
    double right_wheel_velocity;
    std::chrono::steady_clock::time_point timestamp;
};

struct LidarData {
    double x;
    double y;
    double theta;
    double confidence;  // Confidence value between 0.0 and 1.0
    std::chrono::steady_clock::time_point timestamp;
};

struct OdometryData {
    double x;
    double y;
    double theta;
    double linear_velocity;
    double angular_velocity;
    std::chrono::steady_clock::time_point timestamp;
};

class WheelEncoderSimulator {
public:
    WheelEncoderSimulator(double wheel_base = 0.5, double wheel_radius = 0.1);
    
    void updateRobotState(double linear_velocity, double angular_velocity);
    WheelEncoderData getEncoderData();
    
private:
    double wheel_base_;
    double wheel_radius_;
    double left_wheel_velocity_;
    double right_wheel_velocity_;
    std::mt19937 rng_;
    std::normal_distribution<double> noise_dist_;
};

class LidarSimulator {
public:
    LidarSimulator(double update_rate = 10.0, double noise_std = 0.05, 
                   double lag_mean_ms = 0.0, double lag_std_ms = 0.0);
    
    void updateOdometry(const OdometryData& odom_data);
    LidarData getLidarData();
    
    // Sensor control methods
    void enable();
    void disable();
    bool isEnabled() const { return enabled_; }
    
    // Confidence calculation
    double calculateConfidence(double x, double y) const;
    
private:
    double update_rate_;
    double x_;
    double y_;
    double theta_;
    bool enabled_;
    std::chrono::steady_clock::time_point last_update_;
    
    // Noise configuration
    double noise_std_;
    std::mt19937 rng_;
    std::normal_distribution<double> noise_dist_;
    
    // Lag configuration
    double lag_mean_ms_;
    double lag_std_ms_;
    std::normal_distribution<double> lag_dist_;
    
    // Internal state for lag simulation
    std::chrono::steady_clock::time_point last_pose_update_;
    double cached_x_, cached_y_, cached_theta_;
    
    // Odometry integration state
    double last_odom_x_, last_odom_y_, last_odom_theta_;
    std::chrono::steady_clock::time_point last_odom_timestamp_;
    bool has_previous_odom_;
};

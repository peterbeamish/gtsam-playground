#include "sensor_simulator.h"
#include <iostream>

WheelEncoderSimulator::WheelEncoderSimulator(double wheel_base, double wheel_radius)
    : wheel_base_(wheel_base), wheel_radius_(wheel_radius),
      left_wheel_velocity_(0.0), right_wheel_velocity_(0.0),
      rng_(std::random_device{}()), noise_dist_(0.0, 0.01) {
}

void WheelEncoderSimulator::updateRobotState(double linear_velocity, double angular_velocity) {
    // Convert robot velocities to wheel velocities
    // v_left = (v - w * L/2) / r
    // v_right = (v + w * L/2) / r
    // where v is linear velocity, w is angular velocity, L is wheel base, r is wheel radius
    
    double left_velocity = (linear_velocity - angular_velocity * wheel_base_ / 2.0) / wheel_radius_;
    double right_velocity = (linear_velocity + angular_velocity * wheel_base_ / 2.0) / wheel_radius_;
    
    // Add noise
    left_wheel_velocity_ = left_velocity + noise_dist_(rng_);
    right_wheel_velocity_ = right_velocity + noise_dist_(rng_);
}

WheelEncoderData WheelEncoderSimulator::getEncoderData() {
    WheelEncoderData data;
    data.left_wheel_velocity = left_wheel_velocity_;
    data.right_wheel_velocity = right_wheel_velocity_;
    data.timestamp = std::chrono::steady_clock::now();
    return data;
}

LidarSimulator::LidarSimulator(double update_rate, double noise_std, 
                               double lag_mean_ms, double lag_std_ms)
    : update_rate_(update_rate), x_(0.0), y_(0.0), theta_(0.0), enabled_(true),
      last_update_(std::chrono::steady_clock::now()),
      noise_std_(noise_std), rng_(std::random_device{}()), 
      noise_dist_(0.0, noise_std),
      lag_mean_ms_(lag_mean_ms), lag_std_ms_(lag_std_ms),
      lag_dist_(lag_mean_ms, lag_std_ms),
      last_pose_update_(std::chrono::steady_clock::now()),
      cached_x_(0.0), cached_y_(0.0), cached_theta_(0.0) {
}

void LidarSimulator::updateRobotPose(double x, double y, double theta) {
    // Store the latest pose for lag simulation
    cached_x_ = x;
    cached_y_ = y;
    cached_theta_ = theta;
    last_pose_update_ = std::chrono::steady_clock::now();
    
    // Update current pose (for immediate use when no lag)
    x_ = x;
    y_ = y;
    theta_ = theta;
}

LidarData LidarSimulator::getLidarData() {
    LidarData data;
    
    if (!enabled_) {
        // Return last known position when disabled (no new updates)
        data.x = x_;
        data.y = y_;
        data.theta = theta_;
        data.timestamp = last_update_; // Use last update time
        return data;
    }
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update_).count();
    
    // Only update at the specified rate when enabled
    if (elapsed >= (1000.0 / update_rate_)) {
        last_update_ = now;
        
        // Simulate lag by using pose data from the past
        double lag_ms = lag_dist_(rng_); // Generate random lag
        auto lagged_time = now - std::chrono::milliseconds(static_cast<int>(lag_ms));
        
        // Use cached pose if lag is within reasonable bounds, otherwise use current pose
        auto time_since_cached = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_pose_update_).count();
        
        if (lag_ms > 0 && lag_ms < time_since_cached) {
            // Use lagged pose (simulate delayed measurement)
            data.x = cached_x_ + noise_dist_(rng_);
            data.y = cached_y_ + noise_dist_(rng_);
            data.theta = cached_theta_ + noise_dist_(rng_);
        } else {
            // Use current pose (no lag or lag exceeds available history)
            data.x = x_ + noise_dist_(rng_);
            data.y = y_ + noise_dist_(rng_);
            data.theta = theta_ + noise_dist_(rng_);
        }
        
        data.timestamp = now;
    } else {
        // Return cached data if not time for update yet
        data.x = x_;
        data.y = y_;
        data.theta = theta_;
        data.timestamp = last_update_;
    }
    
    return data;
}

void LidarSimulator::enable() {
    enabled_ = true;
    std::cout << "LiDAR sensor enabled" << std::endl;
}

void LidarSimulator::disable() {
    enabled_ = false;
    std::cout << "LiDAR sensor disabled" << std::endl;
}

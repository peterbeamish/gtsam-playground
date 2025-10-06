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
    
    // Only add noise if there's actual movement (avoid phantom movement when stopped)
    if (std::abs(linear_velocity) > 0.001 || std::abs(angular_velocity) > 0.001) {
        left_wheel_velocity_ = left_velocity + noise_dist_(rng_);
        right_wheel_velocity_ = right_velocity + noise_dist_(rng_);
    } else {
        left_wheel_velocity_ = left_velocity;  // No noise when stopped
        right_wheel_velocity_ = right_velocity;
    }
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
      cached_x_(0.0), cached_y_(0.0), cached_theta_(0.0),
      last_odom_x_(0.0), last_odom_y_(0.0), last_odom_theta_(0.0),
      last_odom_timestamp_(std::chrono::steady_clock::now()),
      has_previous_odom_(false) {
}

void LidarSimulator::updateOdometry(const OdometryData& odom_data) {
    auto now = std::chrono::steady_clock::now();
    
    if (has_previous_odom_) {
        // Calculate time delta
        auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_odom_timestamp_).count() / 1000.0; // Convert to seconds
        
        if (dt > 0.001) { // Only integrate if there's meaningful time difference
            // Integrate odometry to get absolute position
            // Use the odometry position directly (it's already integrated)
            x_ = odom_data.x;
            y_ = odom_data.y;
            theta_ = odom_data.theta;
            
            // Store for lag simulation
            cached_x_ = x_;
            cached_y_ = y_;
            cached_theta_ = theta_;
            last_pose_update_ = now;
        }
    } else {
        // First odometry reading - initialize position
        x_ = odom_data.x;
        y_ = odom_data.y;
        theta_ = odom_data.theta;
        cached_x_ = x_;
        cached_y_ = y_;
        cached_theta_ = theta_;
        last_pose_update_ = now;
        has_previous_odom_ = true;
    }
    
    // Store current odometry for next integration
    last_odom_x_ = odom_data.x;
    last_odom_y_ = odom_data.y;
    last_odom_theta_ = odom_data.theta;
    last_odom_timestamp_ = now;
}

LidarData LidarSimulator::getLidarData() {
    LidarData data;
    
    if (!enabled_) {
        // Return last known position when disabled (no new updates)
        data.x = x_;
        data.y = y_;
        data.theta = theta_;
        data.confidence = calculateConfidence(x_, y_);
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
        
        // Calculate confidence based on location
        data.confidence = calculateConfidence(data.x, data.y);
        data.timestamp = now;
    } else {
        // Return cached data if not time for update yet
        data.x = x_;
        data.y = y_;
        data.theta = theta_;
        data.confidence = calculateConfidence(x_, y_);
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

double LidarSimulator::calculateConfidence(double x, double y) const {
    // Define a low-confidence area (not in center, max 1/6 of total area)
    // Let's create a rectangular area in the top-right quadrant
    // Map bounds: approximately -10 to +10 in both x and y
    // Low confidence area: x from 3 to 7, y from 3 to 7 (4x4 = 16 units, ~1/6 of 20x20 map)
    
    const double low_conf_x_min = 3.0;
    const double low_conf_x_max = 7.0;
    const double low_conf_y_min = 3.0;
    const double low_conf_y_max = 7.0;
    
    // Check if we're in the low-confidence area
    if (x >= low_conf_x_min && x <= low_conf_x_max && 
        y >= low_conf_y_min && y <= low_conf_y_max) {
        return 0.2;  // 20% confidence in low-confidence area
    }
    
    return 1.0;  // 100% confidence in normal areas
}

#include "odometry_processor.h"
#include <cmath>

OdometryProcessor::OdometryProcessor(double wheel_base, double wheel_radius)
    : wheel_base_(wheel_base), wheel_radius_(wheel_radius),
      last_left_velocity_(0.0), last_right_velocity_(0.0),
      first_update_(true), x_(0.0), y_(0.0), theta_(0.0) {
}

OdometryData OdometryProcessor::processEncoderData(const WheelEncoderData& encoder_data) {
    OdometryData odom_data;
    
    if (first_update_) {
        last_left_velocity_ = encoder_data.left_wheel_velocity;
        last_right_velocity_ = encoder_data.right_wheel_velocity;
        last_timestamp_ = encoder_data.timestamp;
        first_update_ = false;
        
        odom_data.x = x_;
        odom_data.y = y_;
        odom_data.theta = theta_;
        odom_data.linear_velocity = 0.0;
        odom_data.angular_velocity = 0.0;
        odom_data.timestamp = encoder_data.timestamp;
        return odom_data;
    }
    
    // Calculate time delta
    auto time_delta = std::chrono::duration_cast<std::chrono::milliseconds>(
        encoder_data.timestamp - last_timestamp_).count() / 1000.0;
    
    if (time_delta <= 0) {
        time_delta = 0.001; // Minimum time delta to avoid division by zero
    }
    
    // Convert wheel velocities to robot velocities
    // v = (v_left + v_right) * r / 2
    // w = (v_right - v_left) * r / L
    double linear_velocity = (encoder_data.left_wheel_velocity + encoder_data.right_wheel_velocity) 
                           * wheel_radius_ / 2.0;
    double angular_velocity = (encoder_data.right_wheel_velocity - encoder_data.left_wheel_velocity) 
                             * wheel_radius_ / wheel_base_;
    
    // Integrate to get position
    double delta_theta = angular_velocity * time_delta;
    double delta_x = linear_velocity * std::cos(theta_ + delta_theta / 2.0) * time_delta;
    double delta_y = linear_velocity * std::sin(theta_ + delta_theta / 2.0) * time_delta;
    
    x_ += delta_x;
    y_ += delta_y;
    theta_ += delta_theta;
    
    // Normalize angle
    while (theta_ > M_PI) theta_ -= 2.0 * M_PI;
    while (theta_ < -M_PI) theta_ += 2.0 * M_PI;
    
    odom_data.x = x_;
    odom_data.y = y_;
    odom_data.theta = theta_;
    odom_data.linear_velocity = linear_velocity;
    odom_data.angular_velocity = angular_velocity;
    odom_data.timestamp = encoder_data.timestamp;
    
    // Update for next iteration
    last_left_velocity_ = encoder_data.left_wheel_velocity;
    last_right_velocity_ = encoder_data.right_wheel_velocity;
    last_timestamp_ = encoder_data.timestamp;
    
    return odom_data;
}

void OdometryProcessor::reset() {
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;
    first_update_ = true;
}

#pragma once

#include "../sensors/sensor_simulator.h"

class OdometryProcessor {
public:
    OdometryProcessor(double wheel_base = 0.5, double wheel_radius = 0.1);
    
    OdometryData processEncoderData(const WheelEncoderData& encoder_data);
    void reset();
    
private:
    double wheel_base_;
    double wheel_radius_;
    double last_left_velocity_;
    double last_right_velocity_;
    std::chrono::steady_clock::time_point last_timestamp_;
    bool first_update_;
    
    // Robot state
    double x_;
    double y_;
    double theta_;
};

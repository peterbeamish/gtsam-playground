#pragma once

#include <crow.h>
#include <thread>
#include <atomic>
#include <mutex>
#include <string>

struct SensorData {
    double left_wheel_velocity;
    double right_wheel_velocity;
    double lidar_x;
    double lidar_y;
    double lidar_theta;
    double odom_x;
    double odom_y;
    double odom_theta;
    double gtsam_x;
    double gtsam_y;
    double gtsam_theta;
    double gtsam_cov_xx;
    double gtsam_cov_yy;
    double gtsam_cov_tt;
};

class WebUI {
public:
    WebUI(int port = 8080);
    ~WebUI();
    
    void start();
    void stop();
    void updateSensorData(const SensorData& data);
    
private:
    crow::SimpleApp app_;
    int port_;
    std::thread server_thread_;
    std::atomic<bool> running_;
    
    SensorData current_data_;
    std::mutex data_mutex_;
    
    void setupRoutes();
    std::string generateHTML();
    std::string generateJSON();
};

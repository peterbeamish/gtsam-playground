#pragma once

#include <thread>
#include <atomic>
#include <mutex>
#include <string>
#include <map>
#include <sstream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <fcntl.h>
#include <functional>
#include <memory>
#include <vector>

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
    bool lidar_enabled;
    
    // Ghost robot data for alternative sensor configurations
    double odom_only_x;
    double odom_only_y;
    double odom_only_theta;
    double odom_only_cov_xx;
    double odom_only_cov_yy;
    double odom_only_cov_tt;
    
    double lidar_only_x;
    double lidar_only_y;
    double lidar_only_theta;
    double lidar_only_cov_xx;
    double lidar_only_cov_yy;
    double lidar_only_cov_tt;
    
    // Performance data
    std::vector<double> gtsam_times;
    std::vector<double> odometry_times;
    std::vector<double> lidar_times;
    std::vector<double> total_times;
    double avg_gtsam_time;
    double max_gtsam_time;
    double min_gtsam_time;
};

class SimpleWebUI {
public:
    SimpleWebUI(int port = 8080);
    ~SimpleWebUI();
    
    void start();
    void stop();
    void updateSensorData(const SensorData& data);
    void setControlCallback(std::function<void(const std::string&)> callback);
    
private:
    int port_;
    int server_socket_;
    std::thread server_thread_;
    std::atomic<bool> running_;
    
    SensorData current_data_;
    std::mutex data_mutex_;
    std::function<void(const std::string&)> control_callback_;
    
    void serverLoop();
    std::string generateHTML();
    std::string generateJSON();
    std::string handleRequest(const std::string& request);
    void sendResponse(int client_socket, const std::string& response);
    void handleControlCommand(const std::string& command);
};

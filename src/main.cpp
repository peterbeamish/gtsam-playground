#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <signal.h>
#include <memory>

#include "sensors/sensor_simulator.h"
#include "odometry/odometry_processor.h"
#include "robot/robot_controller.h"
#include "web_ui/simple_web_ui.h"
#include "gtsam_integration/gtsam_integrator.h"
#include "performance/performance_monitor.h"

class WebRobotSimulation {
public:
    WebRobotSimulation() : running_(false), lidar_enabled_(true) {
        // Initialize components
        wheel_encoder_ = std::make_unique<WheelEncoderSimulator>();
        // LiDAR with configurable noise and lag: 10Hz, 0.01m noise, 50ms mean lag with 20ms std
        lidar_ = std::make_unique<LidarSimulator>(10.0, 0.01, 50.0, 20.0);
        odometry_processor_ = std::make_unique<OdometryProcessor>();
        robot_controller_ = std::make_unique<RobotController>();
        gtsam_integrator_ = std::make_unique<GTSAMIntegrator>(50); // 50 pose sliding window
        performance_monitor_ = std::make_unique<PerformanceMonitor>(100);
        web_ui_ = std::make_unique<SimpleWebUI>();
        
        // Initialize robot in STOP state
        robot_controller_->setControlCommand(ControlCommand::STOP);
        
        // Connect robot controller to web UI via callback
        web_ui_->setControlCallback([this](const std::string& command) {
            if (command == "forward") {
                robot_controller_->setControlCommand(ControlCommand::FORWARD);
            } else if (command == "backward") {
                robot_controller_->setControlCommand(ControlCommand::BACKWARD);
            } else if (command == "left") {
                robot_controller_->setControlCommand(ControlCommand::LEFT);
            } else if (command == "right") {
                robot_controller_->setControlCommand(ControlCommand::RIGHT);
            } else if (command == "stop") {
                robot_controller_->setControlCommand(ControlCommand::STOP);
            } else if (command == "toggle_lidar") {
                if (lidar_enabled_) {
                    lidar_->disable();
                    lidar_enabled_ = false;
                } else {
                    lidar_->enable();
                    lidar_enabled_ = true;
                }
                std::cout << "LiDAR " << (lidar_enabled_ ? "enabled" : "disabled") << std::endl;
            }
        });
    }
    
    void start() {
        running_ = true;
        
        // Start web UI
        web_ui_->start();
        
        // Start main simulation loop
        simulation_thread_ = std::thread(&WebRobotSimulation::simulationLoop, this);
        
        std::cout << "Web Robot simulation started!" << std::endl;
        std::cout << "Open http://localhost:8080 in your browser" << std::endl;
        std::cout << "Use W,A,S,D keys in the web interface to control the robot" << std::endl;
        
        // Wait for simulation thread
        if (simulation_thread_.joinable()) {
            simulation_thread_.join();
        }
    }
    
    void stop() {
        running_ = false;
        web_ui_->stop();
    }
    
private:
    std::atomic<bool> running_;
    std::atomic<bool> lidar_enabled_;
    std::thread simulation_thread_;
    
    std::unique_ptr<WheelEncoderSimulator> wheel_encoder_;
    std::unique_ptr<LidarSimulator> lidar_;
    std::unique_ptr<OdometryProcessor> odometry_processor_;
    std::unique_ptr<RobotController> robot_controller_;
    std::unique_ptr<GTSAMIntegrator> gtsam_integrator_;
    std::unique_ptr<PerformanceMonitor> performance_monitor_;
    std::unique_ptr<SimpleWebUI> web_ui_;
    
    // Independent sensor data storage
    OdometryData current_odometry_;
    LidarData current_lidar_;
    
    void simulationLoop() {
        auto last_update = std::chrono::steady_clock::now();
        
        while (running_) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update).count();
            
            if (elapsed >= 50) { // Update at 20 Hz
                updateSimulation();
                last_update = now;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    
    void updateSimulation() {
        Timer total_timer;
        total_timer.start();
        
        // Update robot state (only velocities, position comes from GTSAM)
        RobotState robot_state = robot_controller_->updateState();
        
        // Get current GTSAM estimate for position
        GTSAMEstimate current_gtsam = gtsam_integrator_->getCurrentEstimate();
        
        // Update wheel encoders with robot velocities
        wheel_encoder_->updateRobotState(robot_state.linear_velocity, robot_state.angular_velocity);
        
        // Get encoder data
        WheelEncoderData encoder_data = wheel_encoder_->getEncoderData();
        
        // Process odometry
        Timer odom_timer;
        odom_timer.start();
        OdometryData odom_data = odometry_processor_->processEncoderData(encoder_data);
        odom_timer.stop();
        performance_monitor_->recordOdometryTime(odom_timer.getElapsedMilliseconds());
        
        // Update LiDAR with odometry data (LiDAR simulates absolute position from odometry)
        lidar_->updateOdometry(odom_data);
        
        // Get LiDAR data (at lower rate with noise)
        LidarData lidar_data = lidar_->getLidarData();
        
        // Store independent sensor data
        current_odometry_ = odom_data;
        current_lidar_ = lidar_data;
        
        // Add measurements to GTSAM for fusion
        gtsam_integrator_->addOdometryMeasurement(odom_data);
        gtsam_integrator_->addLidarMeasurement(lidar_data);
        
        // Get GTSAM fusion estimate with timing
        Timer gtsam_timer;
        gtsam_timer.start();
        GTSAMEstimate gtsam_estimate = gtsam_integrator_->getCurrentEstimate();
        gtsam_timer.stop();
        performance_monitor_->recordGTSAMTime(gtsam_timer.getElapsedMilliseconds());
        
        // Update web UI with independent sensor data
        SensorData sensor_data;
        sensor_data.left_wheel_velocity = encoder_data.left_wheel_velocity;
        sensor_data.right_wheel_velocity = encoder_data.right_wheel_velocity;
        
        // Independent LiDAR data
        sensor_data.lidar_x = current_lidar_.x;
        sensor_data.lidar_y = current_lidar_.y;
        sensor_data.lidar_theta = current_lidar_.theta;
        sensor_data.lidar_confidence = current_lidar_.confidence;
        sensor_data.lidar_enabled = lidar_->isEnabled();
        
        // Independent odometry data
        sensor_data.odom_x = current_odometry_.x;
        sensor_data.odom_y = current_odometry_.y;
        sensor_data.odom_theta = current_odometry_.theta;
        
        // GTSAM fusion estimate
        sensor_data.gtsam_x = gtsam_estimate.x;
        sensor_data.gtsam_y = gtsam_estimate.y;
        sensor_data.gtsam_theta = gtsam_estimate.theta;
        sensor_data.gtsam_cov_xx = gtsam_estimate.covariance_xx;
        sensor_data.gtsam_cov_yy = gtsam_estimate.covariance_yy;
        sensor_data.gtsam_cov_tt = gtsam_estimate.covariance_tt;
        
        // Set independent sensor data as "only" estimates (no separate GTSAM processing)
        sensor_data.odom_only_x = current_odometry_.x;
        sensor_data.odom_only_y = current_odometry_.y;
        sensor_data.odom_only_theta = current_odometry_.theta;
        sensor_data.odom_only_cov_xx = 0.0;  // No covariance for independent data
        sensor_data.odom_only_cov_yy = 0.0;
        sensor_data.odom_only_cov_tt = 0.0;
        
        sensor_data.lidar_only_x = current_lidar_.x;
        sensor_data.lidar_only_y = current_lidar_.y;
        sensor_data.lidar_only_theta = current_lidar_.theta;
        sensor_data.lidar_only_cov_xx = 0.0;  // No covariance for independent data
        sensor_data.lidar_only_cov_yy = 0.0;
        sensor_data.lidar_only_cov_tt = 0.0;
        
        // Add performance data
        sensor_data.gtsam_times = performance_monitor_->getGTSAMTimes();
        sensor_data.odometry_times = performance_monitor_->getOdometryTimes();
        sensor_data.lidar_times = performance_monitor_->getLidarTimes();
        sensor_data.total_times = performance_monitor_->getTotalTimes();
        sensor_data.avg_gtsam_time = performance_monitor_->getAverageGTSAMTime();
        sensor_data.max_gtsam_time = performance_monitor_->getMaxGTSAMTime();
        sensor_data.min_gtsam_time = performance_monitor_->getMinGTSAMTime();
        
        // Record total time
        total_timer.stop();
        performance_monitor_->recordTotalTime(total_timer.getElapsedMilliseconds());
        
        web_ui_->updateSensorData(sensor_data);
    }
};

// Global instance for signal handling
std::unique_ptr<WebRobotSimulation> g_simulation;

void signalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ". Shutting down..." << std::endl;
    if (g_simulation) {
        std::cout << "Stopping simulation..." << std::endl;
        g_simulation->stop();
        std::cout << "Simulation stopped." << std::endl;
    }
    std::cout << "Exiting..." << std::endl;
    exit(0);
}

int main() {
    // Set up signal handling
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    try {
        g_simulation = std::make_unique<WebRobotSimulation>();
        g_simulation->start();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}

#pragma once

#include <vector>
#include <chrono>
#include <mutex>
#include <atomic>

class PerformanceMonitor {
public:
    PerformanceMonitor(size_t buffer_size = 100);
    
    // Record execution time for different operations
    void recordGTSAMTime(double milliseconds);
    void recordOdometryTime(double milliseconds);
    void recordLidarTime(double milliseconds);
    void recordTotalTime(double milliseconds);
    
    // Get recent performance data
    std::vector<double> getGTSAMTimes() const;
    std::vector<double> getOdometryTimes() const;
    std::vector<double> getLidarTimes() const;
    std::vector<double> getTotalTimes() const;
    
    // Get statistics
    double getAverageGTSAMTime() const;
    double getMaxGTSAMTime() const;
    double getMinGTSAMTime() const;
    
    // Reset all data
    void reset();
    
private:
    mutable std::mutex mutex_;
    size_t buffer_size_;
    size_t current_index_;
    std::atomic<bool> buffer_full_;
    
    std::vector<double> gtsam_times_;
    std::vector<double> odometry_times_;
    std::vector<double> lidar_times_;
    std::vector<double> total_times_;
    
    void addToBuffer(std::vector<double>& buffer, double value);
};

// Utility class for timing measurements
class Timer {
public:
    Timer();
    ~Timer();
    
    void start();
    void stop();
    double getElapsedMilliseconds() const;
    
private:
    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point end_time_;
    bool is_running_;
};

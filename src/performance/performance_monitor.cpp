#include "performance_monitor.h"
#include <algorithm>
#include <numeric>

PerformanceMonitor::PerformanceMonitor(size_t buffer_size) 
    : buffer_size_(buffer_size), current_index_(0), buffer_full_(false) {
    gtsam_times_.reserve(buffer_size_);
    odometry_times_.reserve(buffer_size_);
    lidar_times_.reserve(buffer_size_);
    total_times_.reserve(buffer_size_);
}

void PerformanceMonitor::recordGTSAMTime(double milliseconds) {
    std::lock_guard<std::mutex> lock(mutex_);
    addToBuffer(gtsam_times_, milliseconds);
}

void PerformanceMonitor::recordOdometryTime(double milliseconds) {
    std::lock_guard<std::mutex> lock(mutex_);
    addToBuffer(odometry_times_, milliseconds);
}

void PerformanceMonitor::recordLidarTime(double milliseconds) {
    std::lock_guard<std::mutex> lock(mutex_);
    addToBuffer(lidar_times_, milliseconds);
}

void PerformanceMonitor::recordTotalTime(double milliseconds) {
    std::lock_guard<std::mutex> lock(mutex_);
    addToBuffer(total_times_, milliseconds);
}

void PerformanceMonitor::addToBuffer(std::vector<double>& buffer, double value) {
    if (buffer.size() < buffer_size_) {
        buffer.push_back(value);
    } else {
        buffer[current_index_] = value;
    }
    
    current_index_ = (current_index_ + 1) % buffer_size_;
    if (current_index_ == 0) {
        buffer_full_ = true;
    }
}

std::vector<double> PerformanceMonitor::getGTSAMTimes() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return gtsam_times_;
}

std::vector<double> PerformanceMonitor::getOdometryTimes() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return odometry_times_;
}

std::vector<double> PerformanceMonitor::getLidarTimes() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return lidar_times_;
}

std::vector<double> PerformanceMonitor::getTotalTimes() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return total_times_;
}

double PerformanceMonitor::getAverageGTSAMTime() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (gtsam_times_.empty()) return 0.0;
    
    double sum = std::accumulate(gtsam_times_.begin(), gtsam_times_.end(), 0.0);
    return sum / gtsam_times_.size();
}

double PerformanceMonitor::getMaxGTSAMTime() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (gtsam_times_.empty()) return 0.0;
    
    return *std::max_element(gtsam_times_.begin(), gtsam_times_.end());
}

double PerformanceMonitor::getMinGTSAMTime() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (gtsam_times_.empty()) return 0.0;
    
    return *std::min_element(gtsam_times_.begin(), gtsam_times_.end());
}

void PerformanceMonitor::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    gtsam_times_.clear();
    odometry_times_.clear();
    lidar_times_.clear();
    total_times_.clear();
    current_index_ = 0;
    buffer_full_ = false;
}

// Timer implementation
Timer::Timer() : is_running_(false) {}

Timer::~Timer() {
    if (is_running_) {
        stop();
    }
}

void Timer::start() {
    start_time_ = std::chrono::steady_clock::now();
    is_running_ = true;
}

void Timer::stop() {
    if (is_running_) {
        end_time_ = std::chrono::steady_clock::now();
        is_running_ = false;
    }
}

double Timer::getElapsedMilliseconds() const {
    auto end_time = is_running_ ? std::chrono::steady_clock::now() : end_time_;
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time_);
    return duration.count() / 1000.0; // Convert to milliseconds
}

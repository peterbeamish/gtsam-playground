#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Vector.h>
#include <memory>
#include <vector>
#include <chrono>
#include <map>

// Forward declarations
struct OdometryData;
struct LidarData;

struct GTSAMEstimate {
    double x;
    double y;
    double theta;
    double covariance_xx;
    double covariance_yy;
    double covariance_tt;
    double error;  // Total graph error
    std::chrono::steady_clock::time_point timestamp;
};

class GTSAMIntegrator {
public:
    GTSAMIntegrator(size_t max_window_size = 100);
    
    void addOdometryMeasurement(const OdometryData& odom_data);
    void addLidarMeasurement(const LidarData& lidar_data);
    GTSAMEstimate getCurrentEstimate();
    void reset();
    
    // Additional utility methods
    void printGraph() const;
    double getGraphError() const;
    size_t getPoseCount() const { return pose_count_; }
    
private:
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initial_estimate_;
    gtsam::Values current_estimate_;
    std::unique_ptr<gtsam::LevenbergMarquardtOptimizer> optimizer_;
    
    size_t pose_count_;
    size_t max_window_size_;
    size_t window_start_index_;  // Starting index of current window
    size_t global_pose_counter_;  // Global counter that never resets
    gtsam::noiseModel::Diagonal::shared_ptr odometry_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr lidar_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr prior_noise_;
    
    // Pose key management
    gtsam::Key getPoseKey(size_t index) const;
    std::map<std::chrono::steady_clock::time_point, gtsam::Key> timestamp_to_key_;
    
    void optimizeGraph();
    void updateInitialEstimate();
    void trimOldPoses();
};

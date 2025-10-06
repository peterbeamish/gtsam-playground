#include "gtsam_integrator.h"
#include "../odometry/odometry_processor.h"
#include "../sensors/sensor_simulator.h"
#include <chrono>
#include <iostream>
#include <iomanip>

GTSAMIntegrator::GTSAMIntegrator() : pose_count_(0) {
    // Initialize noise models using proper GTSAM patterns
    // Prior noise on the first pose (x, y, theta) - sigmas = [0.3m, 0.3m, 0.1rad]
    prior_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector3(0.3, 0.3, 0.1));
    
    // Odometry noise (dx, dy, dtheta) - sigmas = [0.2m, 0.2m, 0.1rad]
    odometry_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector3(0.2, 0.2, 0.1));
    
    // LiDAR measurement noise (x, y, theta) - sigmas = [0.1m, 0.1m, 0.05rad]
    lidar_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector3(0.1, 0.1, 0.05));
}

gtsam::Key GTSAMIntegrator::getPoseKey(size_t index) const {
    return gtsam::Symbol('X', index);
}

void GTSAMIntegrator::addOdometryMeasurement(const OdometryData& odom_data) {
    auto current_time = std::chrono::steady_clock::now();
    
    if (pose_count_ == 0) {
        // Add prior factor for the first pose using proper GTSAM patterns
        gtsam::Pose2 prior_pose(odom_data.x, odom_data.y, odom_data.theta);
        gtsam::Key pose_key = getPoseKey(0);
        
        graph_.add(gtsam::PriorFactor<gtsam::Pose2>(pose_key, prior_pose, prior_noise_));
        initial_estimate_.insert(pose_key, prior_pose);
        timestamp_to_key_[current_time] = pose_key;
        pose_count_++;
        
        std::cout << "Added initial pose: " << prior_pose << std::endl;
        return;
    }
    
    // Add between factor for odometry using proper GTSAM patterns
    gtsam::Pose2 current_pose(odom_data.x, odom_data.y, odom_data.theta);
    gtsam::Key prev_key = getPoseKey(pose_count_ - 1);
    gtsam::Key current_key = getPoseKey(pose_count_);
    
    // Get previous pose from current estimate or initial estimate
    gtsam::Pose2 prev_pose;
    if (current_estimate_.exists(prev_key)) {
        prev_pose = current_estimate_.at<gtsam::Pose2>(prev_key);
    } else {
        prev_pose = initial_estimate_.at<gtsam::Pose2>(prev_key);
    }
    
    // Calculate the relative transformation from previous to current pose
    gtsam::Pose2 odometry_delta = prev_pose.between(current_pose);
    
    // Add between factor
    graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(prev_key, current_key, odometry_delta, odometry_noise_));
    
    // Update initial estimate
    initial_estimate_.insert(current_key, current_pose);
    timestamp_to_key_[current_time] = current_key;
    pose_count_++;
    
    std::cout << "Added odometry factor between poses " << (pose_count_ - 1) 
              << " and " << pose_count_ << ": " << odometry_delta << std::endl;
    
    optimizeGraph();
}

void GTSAMIntegrator::addLidarMeasurement(const LidarData& lidar_data) {
    if (pose_count_ == 0) return; // Need at least one pose
    
    auto current_time = std::chrono::steady_clock::now();
    
    // Check if this is fresh LiDAR data (within last 500ms for 10Hz sensor)
    auto time_since_lidar = std::chrono::duration_cast<std::chrono::milliseconds>(
        current_time - lidar_data.timestamp).count();
    
    if (time_since_lidar > 500) {
        // LiDAR data is stale (sensor likely disabled), don't add to graph
        std::cout << "LiDAR data is stale (" << time_since_lidar << "ms old), skipping measurement" << std::endl;
        return;
    }
    
    // Find the closest pose in time (simplified - use the latest pose)
    gtsam::Key closest_pose_key = getPoseKey(pose_count_ - 1);
    gtsam::Pose2 lidar_pose(lidar_data.x, lidar_data.y, lidar_data.theta);
    
    // Add measurement factor for lidar (soft constraint, not overriding the pose)
    // Use a very weak noise model to avoid over-constraining the optimization
    auto weak_lidar_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.5, 0.5, 0.1));
    graph_.add(gtsam::PriorFactor<gtsam::Pose2>(closest_pose_key, lidar_pose, weak_lidar_noise));
    
    std::cout << "Added LiDAR measurement for pose " << (pose_count_ - 1) 
              << ": " << lidar_pose << std::endl;
    
    optimizeGraph();
}

GTSAMEstimate GTSAMIntegrator::getCurrentEstimate() {
    GTSAMEstimate estimate;
    estimate.timestamp = std::chrono::steady_clock::now();
    
    if (pose_count_ == 0) {
        estimate.x = 0.0;
        estimate.y = 0.0;
        estimate.theta = 0.0;
        estimate.covariance_xx = 0.0;
        estimate.covariance_yy = 0.0;
        estimate.covariance_tt = 0.0;
        estimate.error = 0.0;
        return estimate;
    }
    
    gtsam::Key latest_key = getPoseKey(pose_count_ - 1);
    gtsam::Pose2 latest_pose;
    
    // Get pose from current estimate or initial estimate
    if (current_estimate_.exists(latest_key)) {
        latest_pose = current_estimate_.at<gtsam::Pose2>(latest_key);
    } else {
        latest_pose = initial_estimate_.at<gtsam::Pose2>(latest_key);
    }
    
    estimate.x = latest_pose.x();
    estimate.y = latest_pose.y();
    estimate.theta = latest_pose.theta();
    
    // Calculate covariance using marginals
    try {
        if (current_estimate_.size() > 0) {
            gtsam::Marginals marginals(graph_, current_estimate_);
            gtsam::Matrix covariance = marginals.marginalCovariance(latest_key);
            estimate.covariance_xx = covariance(0, 0);
            estimate.covariance_yy = covariance(1, 1);
            estimate.covariance_tt = covariance(2, 2);
        } else {
            // Use initial estimate for covariance
            gtsam::Marginals marginals(graph_, initial_estimate_);
            gtsam::Matrix covariance = marginals.marginalCovariance(latest_key);
            estimate.covariance_xx = covariance(0, 0);
            estimate.covariance_yy = covariance(1, 1);
            estimate.covariance_tt = covariance(2, 2);
        }
    } catch (const std::exception& e) {
        std::cerr << "Error calculating covariance: " << e.what() << std::endl;
        estimate.covariance_xx = 0.1;
        estimate.covariance_yy = 0.1;
        estimate.covariance_tt = 0.1;
    }
    
    // Calculate total graph error
    estimate.error = getGraphError();
    
    return estimate;
}

void GTSAMIntegrator::optimizeGraph() {
    if (pose_count_ < 2) return;
    
    try {
        // Use Levenberg-Marquardt optimizer with proper parameters
        gtsam::LevenbergMarquardtParams params;
        params.setRelativeErrorTol(1e-5);  // Stop when change in error is small
        params.setMaxIterations(100);      // Limit iterations
        
        optimizer_ = std::make_unique<gtsam::LevenbergMarquardtOptimizer>(graph_, initial_estimate_, params);
        current_estimate_ = optimizer_->optimize();
        
        std::cout << "Graph optimized. Error: " << optimizer_->error() 
                  << ", Iterations: " << optimizer_->iterations() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "GTSAM optimization error: " << e.what() << std::endl;
    }
}

void GTSAMIntegrator::printGraph() const {
    std::cout << "\n=== GTSAM Factor Graph ===" << std::endl;
    std::cout << "Graph size: " << graph_.size() << " factors" << std::endl;
    std::cout << "Pose count: " << pose_count_ << std::endl;
    std::cout << "Initial estimate size: " << initial_estimate_.size() << std::endl;
    std::cout << "Current estimate size: " << current_estimate_.size() << std::endl;
    
    if (graph_.size() > 0) {
        std::cout << "\nGraph structure:" << std::endl;
        for (size_t i = 0; i < graph_.size(); ++i) {
            std::cout << "Factor " << i << ": " << "NonlinearFactor" << std::endl;
        }
    }
    std::cout << "========================\n" << std::endl;
}

double GTSAMIntegrator::getGraphError() const {
    if (current_estimate_.size() > 0) {
        return graph_.error(current_estimate_);
    } else if (initial_estimate_.size() > 0) {
        return graph_.error(initial_estimate_);
    }
    return 0.0;
}

void GTSAMIntegrator::reset() {
    graph_ = gtsam::NonlinearFactorGraph();
    initial_estimate_ = gtsam::Values();
    current_estimate_ = gtsam::Values();
    optimizer_.reset();
    pose_count_ = 0;
    timestamp_to_key_.clear();
    
    std::cout << "GTSAM integrator reset." << std::endl;
}

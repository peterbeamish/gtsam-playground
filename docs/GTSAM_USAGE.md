# GTSAM Integration Guide

This document explains how GTSAM (Georgia Tech Smoothing and Mapping) is integrated and used in our robot simulation project.

## Overview

GTSAM is a C++ library for smoothing and mapping in robotics and vision, utilizing Factor Graphs and Bayes Networks. In our project, we use GTSAM for sensor fusion, combining wheel encoder odometry with LiDAR measurements to provide accurate robot pose estimation.

## Architecture

### Factor Graph Structure

Our GTSAM integration uses a **2D Pose Graph** approach with the following components:

```
Factor Graph:
- PriorFactorPose2: Anchors the first pose at origin
- BetweenFactorPose2: Odometry constraints between consecutive poses
- PriorFactorPose2: LiDAR measurements as soft constraints
```

### Key Components

1. **Pose Variables**: Represented as `Pose2` (x, y, theta) using symbolic keys `X0`, `X1`, `X2`, etc.
2. **Odometry Factors**: Model relative motion between consecutive robot poses
3. **LiDAR Factors**: Provide absolute position measurements as soft constraints
4. **Optimization**: Levenberg-Marquardt algorithm for factor graph optimization

## Implementation Details

### Noise Models

We define three noise models following GTSAM best practices:

```cpp
// Prior noise on the first pose (x, y, theta)
prior_noise_ = gtsam::noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));

// Odometry noise (dx, dy, dtheta) 
odometry_noise_ = gtsam::noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));

// LiDAR measurement noise (x, y, theta)
lidar_noise_ = gtsam::noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.05));
```

**Rationale**:
- **Prior noise**: Higher uncertainty for initial pose (0.3m, 0.3m, 0.1rad)
- **Odometry noise**: Moderate uncertainty for wheel encoder measurements (0.2m, 0.2m, 0.1rad)
- **LiDAR noise**: Lower uncertainty for more precise LiDAR measurements (0.1m, 0.1m, 0.05rad)

### Factor Types

#### 1. Prior Factor (Initial Pose)
```cpp
graph_.add(gtsam::PriorFactorPose2(pose_key, prior_pose, prior_noise_));
```
- Anchors the first robot pose at a known location
- Provides absolute reference for the entire trajectory
- Uses `PriorFactorPose2` for 2D pose constraints

#### 2. Between Factor (Odometry)
```cpp
graph_.add(gtsam::BetweenFactorPose2(prev_key, current_key, odometry_delta, odometry_noise_));
```
- Models relative motion between consecutive poses
- Uses wheel encoder data to estimate pose changes
- Creates a chain of odometry constraints

#### 3. Prior Factor (LiDAR Measurements)
```cpp
graph_.add(gtsam::PriorFactorPose2(closest_pose_key, lidar_pose, lidar_noise_));
```
- Provides absolute position measurements
- Acts as soft constraints to correct odometry drift
- Applied to the most recent pose (simplified approach)

### Optimization Process

```cpp
gtsam::LevenbergMarquardtParams params;
params.setRelativeErrorTol(1e-5);  // Convergence tolerance
params.setMaxIterations(100);      // Maximum iterations

optimizer_ = std::make_unique<gtsam::LevenbergMarquardtOptimizer>(graph_, initial_estimate_, params);
current_estimate_ = optimizer_->optimize();
```

**Optimization Strategy**:
- **Algorithm**: Levenberg-Marquardt (robust for nonlinear problems)
- **Convergence**: Stop when error change < 1e-5
- **Iterations**: Maximum 100 iterations
- **Initialization**: Use odometry-based pose estimates

## Data Flow

### 1. Odometry Integration
```
Wheel Encoders → OdometryProcessor → OdometryData → GTSAMIntegrator
```

**Process**:
1. Wheel encoders provide left/right velocities
2. `OdometryProcessor` converts to robot pose changes
3. `GTSAMIntegrator` adds `BetweenFactorPose2` to graph
4. Graph is optimized to update pose estimates

### 2. LiDAR Integration
```
LiDAR Sensor → LidarData → GTSAMIntegrator
```

**Process**:
1. LiDAR provides absolute position measurements
2. `GTSAMIntegrator` adds `PriorFactorPose2` as soft constraint
3. Graph optimization corrects odometry drift
4. Updated pose estimates are returned

### 3. Pose Estimation
```
GTSAM Factor Graph → Optimization → Pose Estimates + Covariances
```

**Output**:
- **Pose**: (x, y, theta) with uncertainty
- **Covariance**: Uncertainty in each dimension
- **Error**: Total factor graph error

## Key Features

### LidarSimulator Configuration

The `LidarSimulator` now supports configurable noise and lag parameters:

```cpp
// Constructor parameters:
// update_rate: Hz (default: 10.0)
// noise_std: Standard deviation of measurement noise in meters (default: 0.05)
// lag_mean_ms: Mean lag in milliseconds (default: 0.0)
// lag_std_ms: Standard deviation of lag randomness (default: 0.0)

LidarSimulator lidar(10.0, 0.1, 50.0, 20.0);
// 10Hz update rate, 0.1m noise, 50ms mean lag with 20ms randomness
```

**Lag Simulation**:
- Simulates realistic sensor delays where measurements reflect past robot positions
- Random lag adds variability to simulate network delays or processing time
- Helps test GTSAM's robustness to temporal misalignment between sensors

### Symbolic Key Management
```cpp
gtsam::Key GTSAMIntegrator::getPoseKey(size_t index) const {
    return gtsam::Symbol('X', index);
}
```
- Uses symbolic keys (`X0`, `X1`, `X2`, etc.) instead of integer indices
- More readable and maintainable code
- Follows GTSAM best practices

### Robust Error Handling
```cpp
try {
    gtsam::Marginals marginals(graph_, current_estimate_);
    gtsam::Matrix covariance = marginals.marginalCovariance(latest_key);
    // Extract covariance diagonal elements
} catch (const std::exception& e) {
    std::cerr << "Error calculating covariance: " << e.what() << std::endl;
    // Fallback to default uncertainty values
}
```

### Debugging and Monitoring
```cpp
void GTSAMIntegrator::printGraph() const {
    std::cout << "Graph size: " << graph_.size() << " factors" << std::endl;
    std::cout << "Pose count: " << pose_count_ << std::endl;
    // Print factor details for debugging
}
```

## Usage Examples

### Basic Usage
```cpp
GTSAMIntegrator integrator;

// Add initial pose
OdometryData initial_odom = {0.0, 0.0, 0.0};
integrator.addOdometryMeasurement(initial_odom);

// Add odometry measurements
OdometryData odom = {1.0, 0.0, 0.0};  // Move forward 1m
integrator.addOdometryMeasurement(odom);

// Add LiDAR measurement
LidarData lidar = {1.05, 0.02, 0.01};  // Slight correction
integrator.addLidarMeasurement(lidar);

// Get current estimate
GTSAMEstimate estimate = integrator.getCurrentEstimate();
std::cout << "Pose: (" << estimate.x << ", " << estimate.y << ", " << estimate.theta << ")" << std::endl;
std::cout << "Error: " << estimate.error << std::endl;
```

### Advanced Usage
```cpp
// Print graph structure for debugging
integrator.printGraph();

// Get graph error
double error = integrator.getGraphError();

// Reset for new trajectory
integrator.reset();
```

## Performance Considerations

### Optimization Frequency
- **Current**: Optimize after every measurement
- **Alternative**: Batch optimization for better performance
- **Trade-off**: Accuracy vs. computational cost

### Memory Management
- **Pose Storage**: Linear growth with trajectory length
- **Factor Storage**: Linear growth with measurements
- **Covariance**: Computed on-demand to save memory

### Real-time Constraints
- **Optimization Time**: ~1-10ms for small graphs
- **Memory Usage**: ~1KB per pose + factors
- **Scalability**: Suitable for trajectories up to ~1000 poses

## Future Enhancements

### Potential Improvements

1. **Loop Closure Detection**
   ```cpp
   // Add loop closure factors when robot revisits areas
   graph_.add(gtsam::BetweenFactorPose2(pose_i, pose_j, loop_transform, loop_noise));
   ```

2. **Landmark-based SLAM**
   ```cpp
   // Add landmark observations
   graph_.add(gtsam::BearingRangeFactor2D(pose_key, landmark_key, bearing, range, measurement_noise));
   ```

3. **IMU Integration**
   ```cpp
   // Add IMU factors for better motion estimation
   graph_.add(gtsam::ImuFactor(pose_i, vel_i, pose_j, vel_j, bias, pim));
   ```

4. **Incremental Optimization**
   ```cpp
   // Use iSAM2 for incremental updates
   gtsam::ISAM2 isam2;
   isam2.update(graph_, initial_estimate_);
   ```

### Configuration Options

- **Noise Models**: Adjustable based on sensor characteristics
- **Optimization Parameters**: Tunable convergence criteria
- **Factor Types**: Extensible for different sensor types
- **Key Management**: Flexible symbolic key schemes

## References

- [GTSAM Documentation](https://gtsam.org/)
- [Factor Graphs for Robot Perception](http://www.cs.cmu.edu/~kaess/pub/Dellaert17fnt.pdf)
- [GTSAM Examples](https://github.com/borglab/gtsam/tree/develop/python/gtsam/examples)
- [Context7 GTSAM Documentation](https://context7.io/)

## Troubleshooting

### Common Issues

1. **Optimization Divergence**
   - Check noise model values
   - Verify initial estimates
   - Adjust convergence parameters

2. **Memory Issues**
   - Limit trajectory length
   - Use incremental optimization
   - Clear old poses periodically

3. **Poor Accuracy**
   - Tune noise models
   - Add more LiDAR measurements
   - Check sensor calibration

### Debug Tools

- `printGraph()`: Inspect factor graph structure
- `getGraphError()`: Monitor optimization progress
- `getCurrentEstimate()`: Check pose estimates and covariances

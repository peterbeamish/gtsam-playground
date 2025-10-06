# GTSAM Robot Simulation

A C++ project that simulates a robot with wheel encoders, LiDAR, and sensor fusion using the GTSAM library. Features both console-based and web-based interfaces for real-time visualization and control.

## Features

### 🤖 **Robot Control & Motion**
- **Wheel Encoders**: Simulated left and right wheel encoders providing velocity feedback
- **Smooth Acceleration Control**: Realistic acceleration/deceleration instead of instant velocity changes
- **Configurable Max Speed**: Adjustable maximum robot speed (0.1 - 3.0 m/s)
- **Configurable Acceleration**: Adjustable acceleration rate (0.1 - 2.0 m/s²)
- **Keyboard Control**: W,A,S,D keys for robot movement with smooth acceleration

### 📡 **Sensor Simulation**
- **LiDAR**: Simulated LiDAR providing positional updates at 10 Hz
- **Speed-Based Lag Simulation**: LiDAR readings experience increased lag at high speeds
- **Configurable Lag Threshold**: Set speed threshold when lag increases (0.5 - 2.0 m/s)
- **Configurable Lag Factor**: Control how much lag increases above threshold (0.5 - 3.0x)
- **Confidence-Based Noise**: Sensor noise scales with confidence levels

### 🧠 **Sensor Fusion & SLAM**
- **GTSAM Integration**: Advanced sensor fusion using GTSAM for SLAM and state estimation
- **Timestamp-Based Fusion**: Intelligent matching of sensor data to poses using timestamps
- **Adaptive Noise Models**: Noise scales based on data age and confidence
- **Sliding Window Optimization**: Efficient processing with configurable window size
- **Odometry Processing**: Converts wheel encoder data to robot odometry

### 🌐 **Web Interface**
- **Real-time Visualization**: Live robot position and sensor data display
- **Interactive Control Panel**: Sliders for real-time parameter adjustment
- **Multi-Sensor Overlay**: Shows odometry, LiDAR, and GTSAM estimates simultaneously
- **Performance Monitoring**: Real-time timing statistics and graphs
- **Ghost Robot Visualization**: Compare different sensor configurations
- **Responsive Design**: Works on desktop and mobile browsers

### 🛠 **Development & Deployment**
- **Docker Environment**: Reproducible build environment with all dependencies
- **Bazel Build System**: Modern C++ build system with dependency management
- **REST API**: JSON API endpoints for settings and control
- **Real-time Updates**: Live parameter updates without restart

## Quick Start

### 🐳 Docker Setup (Recommended)

The Docker environment includes all dependencies (GTSAM, Eigen, Boost) pre-installed:

```bash
# Build the Docker image (includes GTSAM installation)
make build

# Test the build
make test

# Run the full GTSAM simulation
make simulate

# Run the web interface (maps port 8080)
make web
```

**Available Make Commands:**
- `make build` - Build Docker image with all dependencies
- `make test` - Build and test the project
- `make simulate` - Run full GTSAM simulation
- `make web` - Run web interface (maps port 8080)
- `make run` - Enter container interactively
- `make clean` - Remove container and image

### 🌐 Web Interface

The web interface provides real-time visualization and control:

```bash
# Using Docker (recommended)
make web

# Or manually with Docker
docker run --rm -p 8080:8080 -v $(pwd):/workspace -w /workspace gtsam-playground bazel run //src:robot_simulation
```

Then open your browser and go to: **http://localhost:8080**

**Features:**
- **Real-time Control Panel**: Sliders for max speed, acceleration, and LiDAR lag settings
- **Live Sensor Data**: Real-time display of wheel encoders, LiDAR, odometry, and GTSAM estimates
- **Interactive Robot Visualization**: Top-down view with multiple sensor overlays
- **Smooth Robot Control**: Keyboard controls (W,A,S,D) with realistic acceleration
- **Performance Monitoring**: Live timing statistics and performance graphs
- **Ghost Robot Visualization**: Compare odometry-only vs LiDAR-only vs fused estimates
- **LiDAR Toggle**: Enable/disable LiDAR sensor simulation
- **Configurable Parameters**: Real-time adjustment of robot behavior and sensor characteristics

## Project Structure

```
src/
├── sensors/           # Wheel encoder and LiDAR simulation
│   ├── sensor_simulator.h
│   ├── sensor_simulator.cpp
│   └── BUILD
├── odometry/          # Odometry processing from wheel encoders
│   ├── odometry_processor.h
│   ├── odometry_processor.cpp
│   └── BUILD
├── gtsam_integration/ # GTSAM sensor fusion
│   ├── gtsam_integrator.h
│   ├── gtsam_integrator.cpp
│   └── BUILD
├── robot/             # Robot control and state management
│   ├── robot_controller.h
│   ├── robot_controller.cpp
│   └── BUILD
├── web_ui/            # Web interface
│   ├── simple_web_ui.h
│   ├── simple_web_ui.cpp
│   ├── web_ui.h
│   ├── web_ui.cpp
│   └── BUILD
├── main.cpp           # Main application entry point (web interface + GTSAM)
└── BUILD

docs/
├── DOCKER.md          # Docker setup documentation
└── GTSAM_USAGE.md     # GTSAM integration guide

BUILD.system           # System library definitions
Dockerfile             # Docker environment setup
Makefile              # Docker command shortcuts
install-gtsam.sh      # GTSAM installation script
```

## Recent Improvements

### 🚀 **Latest Features (v2.0)**
- **Smooth Robot Control**: Replaced instant velocity changes with realistic acceleration/deceleration
- **Real-time Parameter Tuning**: Web UI sliders for live adjustment of robot behavior
- **Intelligent Sensor Fusion**: GTSAM now uses timestamps to better match sensor data with poses
- **Speed-Based LiDAR Lag**: Realistic simulation of sensor performance degradation at high speeds
- **Adaptive Noise Models**: Sensor uncertainty scales based on data age and confidence
- **Enhanced Web Interface**: Improved visualization with performance monitoring and ghost robots

### 🔄 **How It Works**
1. **User Input**: Keyboard controls (W,A,S,D) or web interface buttons
2. **Robot Controller**: Applies smooth acceleration to reach target velocities
3. **Sensor Simulation**: Wheel encoders and LiDAR generate realistic data with configurable lag
4. **Odometry Processing**: Converts wheel velocities to robot pose estimates
5. **GTSAM Fusion**: Intelligently combines sensor data using timestamps and confidence
6. **Web Visualization**: Real-time display of all sensor data and robot state

## Architecture

The system consists of several interconnected components:

1. **Sensor Simulators**: Generate realistic sensor data with noise and configurable lag
2. **Odometry Processor**: Converts wheel velocities to robot pose
3. **GTSAM Integrator**: Performs advanced sensor fusion and SLAM with timestamp matching
4. **Robot Controller**: Handles user input with smooth acceleration control
5. **Web UI**: Provides real-time visualization, control, and parameter adjustment

## Available Targets

- `//src:robot_simulation` - **Main robot simulation with web interface and GTSAM sensor fusion**

## Customization

### 🎛️ **Real-time Configuration (Web UI)**
- **Max Speed**: 0.1 - 3.0 m/s (affects robot movement speed)
- **Acceleration**: 0.1 - 2.0 m/s² (affects how quickly robot reaches max speed)
- **LiDAR Lag Threshold**: 0.5 - 2.0 m/s (speed when LiDAR lag starts increasing)
- **LiDAR Lag Factor**: 0.5 - 3.0x (how much lag increases above threshold)

### 🔧 **Code-level Configuration**
- Robot wheel base and radius (`WheelEncoderSimulator` constructor)
- Sensor noise characteristics (noise distributions in sensor classes)
- Update rates (simulation loop timing)
- Web UI port and styling (`WebUI` constructor)
- GTSAM noise models (`GTSAMIntegrator` constructor)
- LiDAR update rate and base lag parameters (`LidarSimulator` constructor)

### 📊 **API Endpoints**
- `GET /api/settings` - Retrieve current configuration
- `POST /api/settings` - Update configuration with JSON payload
- `GET /api/sensor_data` - Get real-time sensor data
- `GET /api/control/{command}` - Send robot control commands

## Troubleshooting

### Build Issues

1. **Docker not found**: Install Docker Desktop
2. **Build fails**: Try `make clean` then `make build`
3. **Port 8080 in use**: Stop other services using port 8080
4. **Permission issues**: Ensure Docker has proper permissions

### Runtime Issues

1. **Terminal input not working**: Ensure you're running in a proper terminal
2. **Web UI not accessible**: Check if port 8080 is available
3. **High CPU usage**: Adjust simulation update rates in the code
4. **Container won't start**: Check Docker logs with `make logs`

## Development

### Adding New Sensors

1. Create a new sensor class in `src/sensors/`
2. Add data structures for sensor output
3. Integrate with the main simulation loop
4. Update the web UI to display new data

### Modifying Robot Behavior

1. Edit `src/robot/robot_controller.cpp`
2. Adjust velocity limits and control logic
3. Add new control commands as needed

### Extending GTSAM Integration

1. Modify `src/gtsam_integration/gtsam_integrator.cpp`
2. Add new factor types
3. Adjust noise models
4. Implement additional optimization strategies

## License

This project is for educational and research purposes.

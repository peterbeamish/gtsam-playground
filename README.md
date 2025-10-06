# GTSAM Robot Simulation

A C++ project that simulates a robot with wheel encoders, LiDAR, and sensor fusion using the GTSAM library. Features both console-based and web-based interfaces for real-time visualization and control.

## Features

- **Wheel Encoders**: Simulated left and right wheel encoders providing velocity feedback
- **LiDAR**: Simulated LiDAR providing positional updates at 10 Hz
- **Odometry Processing**: Converts wheel encoder data to robot odometry
- **GTSAM Integration**: Sensor fusion using GTSAM for SLAM and state estimation
- **Web UI**: Real-time visualization and control interface
- **Console Demo**: Interactive console-based robot simulation
- **Robot Control**: Keyboard control using W,A,S,D keys
- **Docker Environment**: Reproducible build environment with all dependencies
- **Bazel Build System**: Modern C++ build system

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
- Real-time sensor data display
- Interactive robot visualization
- Keyboard controls (W,A,S,D)
- Top-down view with multiple sensor overlays
- GTSAM sensor fusion visualization
- LiDAR toggle functionality

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

## Architecture

The system consists of several interconnected components:

1. **Sensor Simulators**: Generate realistic sensor data with noise
2. **Odometry Processor**: Converts wheel velocities to robot pose
3. **GTSAM Integrator**: Performs sensor fusion and SLAM
4. **Robot Controller**: Handles user input and robot motion
5. **Web UI**: Provides real-time visualization and control

## Available Targets

- `//src:robot_simulation` - **Main robot simulation with web interface and GTSAM sensor fusion**

## Customization

You can modify various parameters in the source code:

- Robot wheel base and radius (`WheelEncoderSimulator` constructor)
- Sensor noise characteristics (noise distributions in sensor classes)
- Update rates (simulation loop timing)
- Web UI port and styling (`WebUI` constructor)
- GTSAM noise models (`GTSAMIntegrator` constructor)

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

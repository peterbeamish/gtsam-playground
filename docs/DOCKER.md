# Docker Setup for GTSAM Playground

This directory contains Docker configuration to build and run the GTSAM robot simulation with proper dependencies.

## Quick Start

1. **Build the Docker image:**
   ```bash
   make build
   ```

2. **Enter the container:**
   ```bash
   make run
   ```

3. **Inside the container, run the build script:**
   ```bash
   ./docker-build.sh
   ```

4. **Run the simulation:**
   ```bash
   bazel run //src:robot_simulation
   ```

## Available Make Commands

- `make help` - Show all available commands
- `make build` - Build the Docker image
- `make run` - Run container interactively
- `make shell` - Enter running container shell
- `make stop` - Stop the container
- `make clean` - Remove container and image
- `make rebuild` - Clean and rebuild everything
- `make install` - Install GTSAM inside container
- `make test` - Build and test the project
- `make simulate` - Run the full GTSAM simulation
- `make web` - Run the web interface (maps port 8080)
- `make logs` - Show container logs
- `make ps` - Show running containers

## Manual Steps

If you prefer to do things manually:

1. **Build the image:**
   ```bash
   docker build -t gtsam-playground .
   ```

2. **Run the container:**
   ```bash
   docker run -it --rm -v $(pwd):/workspace -w /workspace gtsam-playground /bin/bash
   ```

3. **Install GTSAM:**
   ```bash
   ./install-gtsam.sh
   ```

4. **Build the project:**
   ```bash
   bazel build //src:robot_simulation
   ```

5. **Run the simulation:**
   ```bash
   bazel run //src:robot_simulation
   ```

## Available Targets

- `//src:robot_simulation` - Full simulation with GTSAM sensor fusion
- `//src:robot_web` - Web interface (simple version)
- `//src:robot_demo` - Console demo
- `//src:robot_simulation_simple` - Simple background simulation

## Docker Image Contents

- **Base:** Ubuntu 22.04
- **Build Tools:** CMake, GCC, Git, Bazel 7.0.0
- **Dependencies:** Eigen3, Boost, GTSAM (built from source)
- **User:** Non-root `developer` user

## Troubleshooting

### GTSAM Installation Issues
- Make sure you have enough disk space (GTSAM build requires ~2GB)
- If build fails, try: `sudo apt-get update && sudo apt-get install -y build-essential cmake`

### Bazel Issues
- If Bazel can't find system libraries, check that GTSAM is installed in `/usr/local`
- Verify with: `ls -la /usr/local/lib/libgtsam*`

### Permission Issues
- The container runs as a non-root user
- If you need root access: `docker-compose run --user root gtsam-dev`

## Development

The workspace is mounted as a volume, so changes to source files are immediately available in the container. You can edit files on your host machine and rebuild inside the container.

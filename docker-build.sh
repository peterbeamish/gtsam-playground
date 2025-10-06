#!/bin/bash

# Build script for GTSAM playground in Docker
set -e

echo "=== GTSAM Playground Docker Build Script ==="

# Check if we're in Docker
if [ ! -f /.dockerenv ]; then
    echo "This script should be run inside the Docker container"
    echo "To build and enter the container, run:"
    echo "  docker-compose build"
    echo "  docker-compose run gtsam-dev"
    exit 1
fi

# Install GTSAM if not already installed
if [ ! -f /usr/local/lib/libgtsam.so ]; then
    echo "GTSAM not found, installing..."
    /workspace/install-gtsam.sh
else
    echo "GTSAM already installed"
fi

# Verify GTSAM installation
echo "Verifying GTSAM installation..."
ls -la /usr/local/lib/libgtsam*
ls -la /usr/local/include/gtsam/

# Build the project
echo "Building GTSAM playground project..."
cd /workspace

# Build with system dependencies
bazel build //src:robot_simulation

echo "=== Build complete! ==="
echo "To run the full simulation with GTSAM:"
echo "  bazel run //src:robot_simulation"
echo ""
echo "To run the simple web version:"
echo "  bazel run //src:robot_web"
echo ""
echo "To run the console demo:"
echo "  bazel run //src:robot_demo"

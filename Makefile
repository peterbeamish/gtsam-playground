# Makefile for GTSAM Playground Docker Environment

# Variables
IMAGE_NAME = gtsam-playground
CONTAINER_NAME = gtsam-dev
WORKSPACE_DIR = $(shell pwd)

# Default target
.PHONY: help
help:
	@echo "GTSAM Playground Docker Commands:"
	@echo ""
	@echo "  make build     - Build the Docker image"
	@echo "  make run       - Run container interactively"
	@echo "  make shell     - Enter running container shell"
	@echo "  make stop      - Stop the container"
	@echo "  make clean     - Remove container and image"
	@echo "  make rebuild   - Clean and rebuild everything"
	@echo "  make install   - Install GTSAM inside container"
	@echo "  make test      - Build and test the project"
	@echo ""

# Build the Docker image
.PHONY: build
build:
	@echo "Building Docker image..."
	docker build -t $(IMAGE_NAME) .

# Run container interactively
.PHONY: run
run:
	@echo "Starting container..."
	docker run -it --rm \
		--name $(CONTAINER_NAME) \
		-v $(WORKSPACE_DIR):/workspace \
		-w /workspace \
		$(IMAGE_NAME) \
		/bin/bash

# Enter running container shell
.PHONY: shell
shell:
	@echo "Entering container shell..."
	docker exec -it $(CONTAINER_NAME) /bin/bash

# Stop the container
.PHONY: stop
stop:
	@echo "Stopping container..."
	-docker stop $(CONTAINER_NAME)

# Remove container and image
.PHONY: clean
clean:
	@echo "Cleaning up..."
	-docker stop $(CONTAINER_NAME) 2>/dev/null || true
	-docker rm $(CONTAINER_NAME) 2>/dev/null || true
	-docker rmi $(IMAGE_NAME) 2>/dev/null || true

# Rebuild everything
.PHONY: rebuild
rebuild: clean build

# Install GTSAM inside container
.PHONY: install
install:
	@echo "Installing GTSAM..."
	docker run --rm \
		-v $(WORKSPACE_DIR):/workspace \
		-w /workspace \
		$(IMAGE_NAME) \
		./install-gtsam.sh

# Build and test the project
.PHONY: test
test:
	@echo "Building and testing project..."
	docker run --rm \
		-v $(WORKSPACE_DIR):/workspace \
		-w /workspace \
		$(IMAGE_NAME) \
		./docker-build.sh

# Run the full simulation
.PHONY: simulate
simulate:
	@echo "Running GTSAM simulation..."
	docker run --rm \
		-v $(WORKSPACE_DIR):/workspace \
		-w /workspace \
		$(IMAGE_NAME) \
		bazel run //src:robot_simulation

# Run the web interface
.PHONY: web
web:
	@echo "Running web interface..."
	docker run --rm \
		-p 8080:8080 \
		-v $(WORKSPACE_DIR):/workspace \
		-w /workspace \
		$(IMAGE_NAME) \
		bazel run //src:robot_simulation

# Show container logs
.PHONY: logs
logs:
	docker logs $(CONTAINER_NAME)

# Show running containers
.PHONY: ps
ps:
	docker ps -a | grep $(CONTAINER_NAME) || echo "No containers found"

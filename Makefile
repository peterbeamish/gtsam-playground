# Makefile for GTSAM Playground Docker Environment

# Variables
IMAGE_NAME = gtsam-playground
CONTAINER_NAME = gtsam-dev
WEB_CONTAINER_NAME = gtsam-web
WORKSPACE_DIR = $(shell pwd)

# Default target
.PHONY: help
help:
	@echo "GTSAM Playground Docker Commands:"
	@echo ""
	@echo "  make build       - Build the Docker image"
	@echo "  make run         - Run container interactively"
	@echo "  make shell       - Enter running container shell"
	@echo "  make stop        - Stop the container"
	@echo "  make clean       - Remove container and image"
	@echo "  make rebuild     - Clean and rebuild everything"
	@echo "  make web         - Run the web simulation (one-time)"
	@echo "  make web-build   - Build the web simulation using named container"
	@echo "  make web-run     - Run the web simulation using named container"
	@echo "  make web-stop    - Stop the web container"
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
	-docker stop $(WEB_CONTAINER_NAME) 2>/dev/null || true
	-docker rm $(WEB_CONTAINER_NAME) 2>/dev/null || true
	-docker rmi $(IMAGE_NAME) 2>/dev/null || true

# Rebuild everything
.PHONY: rebuild
rebuild: clean build

# Build and test the project
.PHONY: test
test:
	@echo "Building and testing project..."
	docker run --rm \
		-v $(WORKSPACE_DIR):/workspace \
		-w /workspace \
		$(IMAGE_NAME) \
		./docker-build.sh

# Run the full simulation (one-time)
.PHONY: simulate
simulate:
	@echo "Running GTSAM simulation..."
	docker run --rm \
		-v $(WORKSPACE_DIR):/workspace \
		-w /workspace \
		$(IMAGE_NAME) \
		bazel run //src:robot_simulation --enable_workspace

# Run the web interface (one-time)
.PHONY: web
web:
	@echo "Running web interface..."
	docker run --rm \
		-p 8080:8080 \
		-v $(WORKSPACE_DIR):/workspace \
		-w /workspace \
		$(IMAGE_NAME) \
		bazel run //src:robot_simulation --enable_workspace

# Build the web simulation using named container for caching
.PHONY: web-build
web-build:
	@echo "Building web simulation with named container..."
	@if ! docker ps -q -f name=$(WEB_CONTAINER_NAME) | grep -q .; then \
		echo "Web container not running. Starting new container..."; \
		docker run -d --name $(WEB_CONTAINER_NAME) \
			-v $(WORKSPACE_DIR):/workspace \
			-w /workspace \
			$(IMAGE_NAME) \
			sleep infinity; \
	else \
		echo "Web container already running. Using existing container..."; \
	fi
	@echo "Building project in container..."
	docker exec $(WEB_CONTAINER_NAME) bazel build //src:robot_simulation --enable_workspace
	@echo "Build complete. Container $(WEB_CONTAINER_NAME) is running for future use."

# Run the web simulation using the named container
.PHONY: web-run
web-run:
	@echo "Running web simulation from named container..."
	@if ! docker ps -q -f name=$(WEB_CONTAINER_NAME) | grep -q .; then \
		echo "Web container not running. Run 'make web-build' first."; \
		exit 1; \
	fi
	docker exec -p 8080:8080 $(WEB_CONTAINER_NAME) bazel run //src:robot_simulation --enable_workspace

# Stop the web container
.PHONY: web-stop
web-stop:
	@echo "Stopping web container..."
	-docker stop $(WEB_CONTAINER_NAME) 2>/dev/null || true
	-docker rm $(WEB_CONTAINER_NAME) 2>/dev/null || true

# Show container logs
.PHONY: logs
logs:
	docker logs $(CONTAINER_NAME)

# Show running containers
.PHONY: ps
ps:
	@echo "Development containers:"
	@docker ps -a | grep $(CONTAINER_NAME) || echo "No development containers found"
	@echo "Web containers:"
	@docker ps -a | grep $(WEB_CONTAINER_NAME) || echo "No web containers found"

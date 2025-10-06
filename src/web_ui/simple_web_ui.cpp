#include "simple_web_ui.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <memory>
#include <functional>

SimpleWebUI::SimpleWebUI(int port) : port_(port), server_socket_(-1), running_(false) {
}

SimpleWebUI::~SimpleWebUI() {
    stop();
}

void SimpleWebUI::start() {
    running_ = true;
    
    // Create socket
    server_socket_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket_ < 0) {
        std::cerr << "Failed to create socket" << std::endl;
        return;
    }
    
    // Set socket options
    int opt = 1;
    setsockopt(server_socket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    // Bind socket
    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port_);
    
    if (bind(server_socket_, (struct sockaddr*)&address, sizeof(address)) < 0) {
        std::cerr << "Failed to bind socket" << std::endl;
        close(server_socket_);
        return;
    }
    
    // Listen
    if (listen(server_socket_, 5) < 0) {
        std::cerr << "Failed to listen" << std::endl;
        close(server_socket_);
        return;
    }
    
    // Start server thread
    server_thread_ = std::thread(&SimpleWebUI::serverLoop, this);
    
    std::cout << "Simple Web UI started on port " << port_ << std::endl;
    std::cout << "Open http://localhost:" << port_ << " in your browser" << std::endl;
}

void SimpleWebUI::stop() {
    running_ = false;
    if (server_socket_ >= 0) {
        close(server_socket_);
        server_socket_ = -1;
    }
    if (server_thread_.joinable()) {
        server_thread_.join();
    }
}

void SimpleWebUI::updateSensorData(const SensorData& data) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_data_ = data;
}

void SimpleWebUI::setControlCallback(std::function<void(const std::string&)> callback) {
    control_callback_ = callback;
}

void SimpleWebUI::serverLoop() {
    while (running_) {
        struct sockaddr_in client_address;
        socklen_t client_len = sizeof(client_address);
        
        int client_socket = accept(server_socket_, (struct sockaddr*)&client_address, &client_len);
        if (client_socket < 0) {
            if (running_) {
                std::cerr << "Failed to accept connection" << std::endl;
            }
            continue;
        }
        
        // Read request
        char buffer[4096];
        int bytes_read = read(client_socket, buffer, sizeof(buffer) - 1);
        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            std::string request(buffer);
            std::string response = handleRequest(request);
            sendResponse(client_socket, response);
        }
        
        close(client_socket);
    }
}

std::string SimpleWebUI::handleRequest(const std::string& request) {
    if (request.find("GET /api/sensor_data") != std::string::npos) {
        return generateJSON();
    } else if (request.find("GET /api/control/") != std::string::npos) {
        // Extract command from URL
        size_t start = request.find("/api/control/") + 13;
        size_t end = request.find(" ", start);
        if (end == std::string::npos) end = request.find("\r", start);
        if (end == std::string::npos) end = request.find("\n", start);
        
        if (start < request.length() && end != std::string::npos) {
            std::string command = request.substr(start, end - start);
            handleControlCommand(command);
        }
        
        return "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nContent-Length: 2\r\n\r\nOK";
    } else {
        return generateHTML();
    }
}

void SimpleWebUI::sendResponse(int client_socket, const std::string& response) {
    send(client_socket, response.c_str(), response.length(), 0);
}

void SimpleWebUI::handleControlCommand(const std::string& command) {
    if (control_callback_) {
        control_callback_(command);
    }
}

std::string SimpleWebUI::generateHTML() {
    std::ostringstream html;
    html << "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n";
    
    html << "<!DOCTYPE html>\n";
    html << "<html>\n";
    html << "<head>\n";
    html << "    <title>Robot Sensor Fusion Dashboard</title>\n";
    html << "    <style>\n";
    html << "        body { font-family: Arial, sans-serif; margin: 20px; background-color: #f0f0f0; }\n";
    html << "        .container { max-width: 1200px; margin: 0 auto; }\n";
    html << "        .header { text-align: center; margin-bottom: 30px; }\n";
    html << "        .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; }\n";
    html << "        .panel { background: white; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }\n";
    html << "        .sensor-data { margin: 10px 0; }\n";
    html << "        .value { font-weight: bold; color: #333; }\n";
    html << "        .canvas-container { text-align: center; margin-top: 20px; }\n";
    html << "        canvas { border: 1px solid #ccc; background: white; }\n";
    html << "        .controls { text-align: center; margin: 20px 0; }\n";
        html << "        .control-btn { margin: 5px; padding: 10px 20px; font-size: 16px; transition: all 0.1s ease; }\n";
        html << "        .control-btn:active { transform: scale(0.95); background-color: #ddd; }\n";
        html << "        .control-btn.pressed { background-color: #4CAF50; color: white; transform: scale(0.95); }\n";
    html << "        .status { text-align: center; margin: 20px 0; }\n";
    html << "        .legend-container { background: white; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); margin: 20px 0; }\n";
    html << "        .legend { display: flex; flex-wrap: wrap; gap: 15px; justify-content: center; }\n";
    html << "        .legend-item { display: flex; align-items: center; gap: 8px; }\n";
    html << "        .legend-color { width: 20px; height: 20px; border-radius: 50%; border: 1px solid #ccc; }\n";
    html << "        .performance-container { background: white; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); margin: 20px 0; }\n";
    html << "        .performance-stats { display: flex; gap: 20px; margin-bottom: 15px; justify-content: center; }\n";
    html << "        .stat-item { display: flex; flex-direction: column; align-items: center; }\n";
    html << "        .stat-label { font-size: 12px; color: #666; }\n";
    html << "        .stat-value { font-size: 18px; font-weight: bold; color: #333; }\n";
    html << "        .performance-graph-container { text-align: center; }\n";
    html << "        #performanceCanvas { border: 1px solid #ddd; background: #fafafa; }\n";
    html << "    </style>\n";
    html << "</head>\n";
    html << "<body>\n";
    html << "    <div class=\"container\">\n";
    html << "        <div class=\"header\">\n";
    html << "            <h1>Robot Sensor Fusion Dashboard</h1>\n";
    html << "            <p>Real-time sensor data and odometry estimation</p>\n";
    html << "        </div>\n";
    html << "        \n";
        html << "        <div class=\"controls\">\n";
        html << "            <button class=\"control-btn\" onclick=\"sendCommand('forward')\">W - Forward</button>\n";
        html << "            <button class=\"control-btn\" onclick=\"sendCommand('backward')\">S - Backward</button>\n";
        html << "            <button class=\"control-btn\" onclick=\"sendCommand('left')\">A - Left</button>\n";
        html << "            <button class=\"control-btn\" onclick=\"sendCommand('right')\">D - Right</button>\n";
        html << "            <button class=\"control-btn\" onclick=\"sendCommand('stop')\">Stop</button>\n";
        html << "            <button class=\"control-btn\" id=\"lidar-toggle\" onclick=\"toggleLidar()\" style=\"background-color: #4CAF50; color: white;\">LiDAR: ON</button>\n";
        html << "            <button class=\"control-btn\" id=\"ghost-odom-toggle\" onclick=\"toggleGhostOdom()\" style=\"background-color: #FF9800; color: white;\">Ghost Odometry: ON</button>\n";
        html << "            <button class=\"control-btn\" id=\"ghost-lidar-toggle\" onclick=\"toggleGhostLidar()\" style=\"background-color: #9C27B0; color: white;\">Ghost LiDAR: ON</button>\n";
        html << "        </div>\n";
    html << "        \n";
    html << "        <div class=\"grid\">\n";
    html << "            <div class=\"panel\">\n";
    html << "                <h3>Wheel Encoders</h3>\n";
    html << "                <div class=\"sensor-data\">\n";
    html << "                    <div>Left Wheel Velocity: <span class=\"value\" id=\"left-wheel\">0.00</span> rad/s</div>\n";
    html << "                    <div>Right Wheel Velocity: <span class=\"value\" id=\"right-wheel\">0.00</span> rad/s</div>\n";
    html << "                </div>\n";
    html << "            </div>\n";
    html << "            \n";
    html << "            <div class=\"panel\">\n";
    html << "                <h3>LiDAR</h3>\n";
    html << "                <div class=\"sensor-data\">\n";
    html << "                    <div>X: <span class=\"value\" id=\"lidar-x\">0.00</span> m</div>\n";
    html << "                    <div>Y: <span class=\"value\" id=\"lidar-y\">0.00</span> m</div>\n";
        html << "                    <div>theta: <span class=\"value\" id=\"lidar-theta\">0.00</span>°</div>\n";
    html << "                    <div>Confidence: <span class=\"value\" id=\"lidar-confidence\">100</span>%</div>\n";
    html << "                </div>\n";
    html << "            </div>\n";
    html << "            \n";
    html << "            <div class=\"panel\">\n";
    html << "                <h3>Odometry</h3>\n";
    html << "                <div class=\"sensor-data\">\n";
    html << "                    <div>X: <span class=\"value\" id=\"odom-x\">0.00</span> m</div>\n";
    html << "                    <div>Y: <span class=\"value\" id=\"odom-y\">0.00</span> m</div>\n";
        html << "                    <div>theta: <span class=\"value\" id=\"odom-theta\">0.00</span>°</div>\n";
    html << "                </div>\n";
    html << "            </div>\n";
    html << "            \n";
    html << "                        <div class=\"panel\">\n";
            html << "                <h3>GTSAM Estimate</h3>\n";
            html << "                <div class=\"sensor-data\">\n";
            html << "                    <div>X: <span class=\"value\" id=\"robot-x\">0.00</span> m</div>\n";
            html << "                    <div>Y: <span class=\"value\" id=\"robot-y\">0.00</span> m</div>\n";
            html << "                    <div>theta: <span class=\"value\" id=\"robot-theta\">0.00</span>°</div>\n";
            html << "                    <div>Cov XX: <span class=\"value\" id=\"cov-xx\">0.00</span></div>\n";
            html << "                    <div>Cov YY: <span class=\"value\" id=\"cov-yy\">0.00</span></div>\n";
            html << "                    <div>Cov theta: <span class=\"value\" id=\"cov-tt\">0.00</span></div>\n";
            html << "                </div>\n";
            html << "            </div>\n";
    html << "        </div>\n";
    html << "        \n";
        html << "        <div class=\"canvas-container\">\n";
        html << "            <canvas id=\"robotCanvas\" width=\"800\" height=\"600\"></canvas>\n";
        html << "        </div>\n";
        html << "        \n";
        html << "        <div class=\"legend-container\">\n";
        html << "            <h3>Robot Legend</h3>\n";
        html << "            <div class=\"legend\">\n";
        html << "                <div class=\"legend-item\">\n";
        html << "                    <div class=\"legend-color\" style=\"background-color: blue;\"></div>\n";
        html << "                    <span>GTSAM Fusion (Main Robot)</span>\n";
        html << "                </div>\n";
        html << "                <div class=\"legend-item\">\n";
        html << "                    <div class=\"legend-color\" style=\"background-color: red;\"></div>\n";
        html << "                    <span>Raw Odometry</span>\n";
        html << "                </div>\n";
        html << "                <div class=\"legend-item\">\n";
        html << "                    <div class=\"legend-color\" style=\"background-color: green;\"></div>\n";
        html << "                    <span>Raw LiDAR</span>\n";
        html << "                </div>\n";
        html << "                <div class=\"legend-item\">\n";
        html << "                    <div class=\"legend-color\" style=\"background-color: orange; opacity: 0.6;\"></div>\n";
        html << "                    <span>Odometry Trail</span>\n";
        html << "                </div>\n";
        html << "                <div class=\"legend-item\">\n";
        html << "                    <div class=\"legend-color\" style=\"background-color: purple; opacity: 0.6;\"></div>\n";
        html << "                    <span>Ghost LiDAR Robot</span>\n";
        html << "                </div>\n";
        html << "            </div>\n";
        html << "        </div>\n";
        html << "        \n";
        html << "        <div class=\"performance-container\">\n";
        html << "            <h3>Performance Monitor</h3>\n";
        html << "            <div class=\"performance-stats\">\n";
        html << "                <div class=\"stat-item\">\n";
        html << "                    <span class=\"stat-label\">Avg GTSAM Time:</span>\n";
        html << "                    <span class=\"stat-value\" id=\"avg-gtsam-time\">0.00</span> ms\n";
        html << "                </div>\n";
        html << "                <div class=\"stat-item\">\n";
        html << "                    <span class=\"stat-label\">Max GTSAM Time:</span>\n";
        html << "                    <span class=\"stat-value\" id=\"max-gtsam-time\">0.00</span> ms\n";
        html << "                </div>\n";
        html << "                <div class=\"stat-item\">\n";
        html << "                    <span class=\"stat-label\">Min GTSAM Time:</span>\n";
        html << "                    <span class=\"stat-value\" id=\"min-gtsam-time\">0.00</span> ms\n";
        html << "                </div>\n";
        html << "            </div>\n";
        html << "            <div class=\"performance-graph-container\">\n";
        html << "                <canvas id=\"performanceCanvas\" width=\"600\" height=\"200\"></canvas>\n";
        html << "            </div>\n";
        html << "        </div>\n";
        html << "        \n";
        html << "        <div class=\"status\">\n";
        html << "            <p>Status: <span id=\"status\">Connected</span></p>\n";
        html << "        </div>\n";
    html << "    </div>\n";
    html << "    \n";
    html << "    <script>\n";
    html << "        let canvas = document.getElementById('robotCanvas');\n";
    html << "        let ctx = canvas.getContext('2d');\n";
    html << "        \n";
    html << "        // Performance monitoring\n";
    html << "        let perfCanvas = document.getElementById('performanceCanvas');\n";
    html << "        let perfCtx = perfCanvas.getContext('2d');\n";
    html << "        \n";
    html << "        // Ghost robot visibility states\n";
    html << "        let showGhostOdom = true;\n";
    html << "        let showGhostLidar = true;\n";
    html << "        \n";
    html << "        function updateSensorData() {\n";
    html << "            fetch('/api/sensor_data')\n";
    html << "                .then(response => response.json())\n";
    html << "                .then(data => {\n";
        html << "                    document.getElementById('left-wheel').textContent = data.left_wheel_velocity.toFixed(2);\n";
        html << "                    document.getElementById('right-wheel').textContent = data.right_wheel_velocity.toFixed(2);\n";
        html << "                    document.getElementById('lidar-x').textContent = data.lidar_x.toFixed(2);\n";
        html << "                    document.getElementById('lidar-y').textContent = data.lidar_y.toFixed(2);\n";
        html << "                    document.getElementById('lidar-theta').textContent = (data.lidar_theta * 180 / Math.PI).toFixed(1);\n";
        html << "                    document.getElementById('lidar-confidence').textContent = (data.lidar_confidence * 100).toFixed(0);\n";
        html << "                    document.getElementById('odom-x').textContent = data.odom_x.toFixed(2);\n";
        html << "                    document.getElementById('odom-y').textContent = data.odom_y.toFixed(2);\n";
        html << "                    document.getElementById('odom-theta').textContent = (data.odom_theta * 180 / Math.PI).toFixed(1);\n";
        html << "                    document.getElementById('robot-x').textContent = data.robot_x.toFixed(2);\n";
        html << "                    document.getElementById('robot-y').textContent = data.robot_y.toFixed(2);\n";
        html << "                    document.getElementById('robot-theta').textContent = (data.robot_theta * 180 / Math.PI).toFixed(1);\n";
        html << "                    document.getElementById('cov-xx').textContent = data.gtsam_cov_xx.toFixed(4);\n";
        html << "                    document.getElementById('cov-yy').textContent = data.gtsam_cov_yy.toFixed(4);\n";
        html << "                    document.getElementById('cov-tt').textContent = data.gtsam_cov_tt.toFixed(4);\n";
        html << "                    \n";
        html << "                    // Update performance stats\n";
        html << "                    document.getElementById('avg-gtsam-time').textContent = data.avg_gtsam_time.toFixed(2);\n";
        html << "                    document.getElementById('max-gtsam-time').textContent = data.max_gtsam_time.toFixed(2);\n";
        html << "                    document.getElementById('min-gtsam-time').textContent = data.min_gtsam_time.toFixed(2);\n";
        html << "                    \n";
        html << "                    // Update performance graph\n";
        html << "                    drawPerformanceGraph(data.gtsam_times, data.odometry_times, data.total_times);\n";
        html << "                    \n";
        html << "                    // Update LiDAR toggle button\n";
        html << "                    const lidarToggle = document.getElementById('lidar-toggle');\n";
        html << "                    if (data.lidar_enabled) {\n";
        html << "                        lidarToggle.textContent = 'LiDAR: ON';\n";
        html << "                        lidarToggle.style.backgroundColor = '#4CAF50';\n";
        html << "                    } else {\n";
        html << "                        lidarToggle.textContent = 'LiDAR: OFF';\n";
        html << "                        lidarToggle.style.backgroundColor = '#f44336';\n";
        html << "                    }\n";
        html << "                    \n";
        html << "                    // Update ghost robot toggle buttons\n";
        html << "                    const ghostOdomToggle = document.getElementById('ghost-odom-toggle');\n";
        html << "                    if (showGhostOdom) {\n";
        html << "                        ghostOdomToggle.textContent = 'Ghost Odometry: ON';\n";
        html << "                        ghostOdomToggle.style.backgroundColor = '#FF9800';\n";
        html << "                    } else {\n";
        html << "                        ghostOdomToggle.textContent = 'Ghost Odometry: OFF';\n";
        html << "                        ghostOdomToggle.style.backgroundColor = '#666';\n";
        html << "                    }\n";
        html << "                    \n";
        html << "                    const ghostLidarToggle = document.getElementById('ghost-lidar-toggle');\n";
        html << "                    if (showGhostLidar) {\n";
        html << "                        ghostLidarToggle.textContent = 'Ghost LiDAR: ON';\n";
        html << "                        ghostLidarToggle.style.backgroundColor = '#9C27B0';\n";
        html << "                    } else {\n";
        html << "                        ghostLidarToggle.textContent = 'Ghost LiDAR: OFF';\n";
        html << "                        ghostLidarToggle.style.backgroundColor = '#666';\n";
        html << "                    }\n";
    html << "                    \n";
    html << "                    drawRobot(data);\n";
    html << "                })\n";
    html << "                .catch(error => {\n";
    html << "                    document.getElementById('status').textContent = 'Disconnected';\n";
    html << "                    console.error('Error:', error);\n";
    html << "                });\n";
    html << "        }\n";
    html << "        \n";
    html << "        function drawRobot(data) {\n";
    html << "            ctx.clearRect(0, 0, canvas.width, canvas.height);\n";
    html << "            \n";
    html << "            // Draw grid\n";
    html << "            ctx.strokeStyle = '#ddd';\n";
    html << "            ctx.lineWidth = 1;\n";
    html << "            for (let i = 0; i < canvas.width; i += 50) {\n";
    html << "                ctx.beginPath();\n";
    html << "                ctx.moveTo(i, 0);\n";
    html << "                ctx.lineTo(i, canvas.height);\n";
    html << "                ctx.stroke();\n";
    html << "            }\n";
    html << "            for (let i = 0; i < canvas.height; i += 50) {\n";
    html << "                ctx.beginPath();\n";
    html << "                ctx.moveTo(0, i);\n";
    html << "                ctx.lineTo(canvas.width, i);\n";
    html << "                ctx.stroke();\n";
    html << "            }\n";
    html << "            \n";
    html << "            // Scale factor for visualization\n";
    html << "            const scale = 50; // pixels per meter\n";
    html << "            const centerX = canvas.width / 2;\n";
    html << "            const centerY = canvas.height / 2;\n";
    html << "            \n";
    html << "            // Draw low-confidence area (red shaded rectangle)\n";
    html << "            ctx.fillStyle = 'rgba(255, 0, 0, 0.2)';\n";
    html << "            ctx.fillRect(centerX + 3 * scale, centerY - 7 * scale, 4 * scale, 4 * scale);\n";
    html << "            ctx.strokeStyle = 'red';\n";
    html << "            ctx.lineWidth = 2;\n";
    html << "            ctx.strokeRect(centerX + 3 * scale, centerY - 7 * scale, 4 * scale, 4 * scale);\n";
    html << "            \n";
    html << "            // Add label for low-confidence area\n";
    html << "            ctx.fillStyle = 'red';\n";
    html << "            ctx.font = '12px Arial';\n";
    html << "            ctx.fillText('Low Confidence Area', centerX + 3 * scale, centerY - 7 * scale - 5);\n";
    html << "            \n";
    html << "                        // Draw GTSAM estimate position (blue circle)\n";
            html << "            ctx.fillStyle = 'blue';\n";
            html << "            ctx.beginPath();\n";
            html << "            ctx.arc(centerX + data.robot_x * scale, centerY - data.robot_y * scale, 8, 0, 2 * Math.PI);\n";
            html << "            ctx.fill();\n";
            html << "            \n";
            html << "            // Draw uncertainty ellipse for GTSAM estimate\n";
            html << "            if (data.gtsam_cov_xx > 0 && data.gtsam_cov_yy > 0) {\n";
            html << "                ctx.strokeStyle = 'blue';\n";
            html << "                ctx.lineWidth = 1;\n";
            html << "                ctx.setLineDash([2, 2]);\n";
            html << "                ctx.beginPath();\n";
            html << "                const uncertainty_scale = Math.sqrt(Math.max(data.gtsam_cov_xx, data.gtsam_cov_yy)) * scale * 2;\n";
            html << "                ctx.ellipse(centerX + data.robot_x * scale, centerY - data.robot_y * scale, uncertainty_scale, uncertainty_scale, 0, 0, 2 * Math.PI);\n";
            html << "                ctx.stroke();\n";
            html << "                ctx.setLineDash([]);\n";
            html << "            }\n";
    html << "            \n";
    html << "            // Draw odometry position\n";
    html << "            ctx.fillStyle = 'red';\n";
    html << "            ctx.beginPath();\n";
    html << "            ctx.arc(centerX + data.odom_x * scale, centerY - data.odom_y * scale, 5, 0, 2 * Math.PI);\n";
    html << "            ctx.fill();\n";
    html << "            \n";
    html << "                        // Draw LiDAR position (only if enabled)\n";
            html << "            if (data.lidar_enabled) {\n";
            html << "                ctx.fillStyle = 'green';\n";
            html << "                ctx.beginPath();\n";
            html << "                ctx.arc(centerX + data.lidar_x * scale, centerY - data.lidar_y * scale, 3, 0, 2 * Math.PI);\n";
            html << "                ctx.fill();\n";
            html << "            } else {\n";
            html << "                // Draw crossed-out LiDAR position when disabled\n";
            html << "                ctx.strokeStyle = 'red';\n";
            html << "                ctx.lineWidth = 2;\n";
            html << "                ctx.beginPath();\n";
            html << "                ctx.moveTo(centerX + data.lidar_x * scale - 5, centerY - data.lidar_y * scale - 5);\n";
            html << "                ctx.lineTo(centerX + data.lidar_x * scale + 5, centerY - data.lidar_y * scale + 5);\n";
            html << "                ctx.moveTo(centerX + data.lidar_x * scale + 5, centerY - data.lidar_y * scale - 5);\n";
            html << "                ctx.lineTo(centerX + data.lidar_x * scale - 5, centerY - data.lidar_y * scale + 5);\n";
            html << "                ctx.stroke();\n";
            html << "            }\n";
    html << "            \n";
    html << "            // Draw ghost robots (odometry-only and LiDAR-only)\n";
    html << "            if (showGhostOdom && data.odom_only_x !== undefined) {\n";
    html << "                // Ghost odometry robot (orange, semi-transparent)\n";
    html << "                ctx.globalAlpha = 0.6;\n";
    html << "                ctx.fillStyle = 'orange';\n";
    html << "                ctx.beginPath();\n";
    html << "                ctx.arc(centerX + data.odom_only_x * scale, centerY - data.odom_only_y * scale, 6, 0, 2 * Math.PI);\n";
    html << "                ctx.fill();\n";
    html << "                \n";
    html << "                // Ghost odometry uncertainty ellipse\n";
    html << "                if (data.odom_only_cov_xx > 0 && data.odom_only_cov_yy > 0) {\n";
    html << "                    ctx.strokeStyle = 'orange';\n";
    html << "                    ctx.lineWidth = 1;\n";
    html << "                    ctx.setLineDash([3, 3]);\n";
    html << "                    ctx.beginPath();\n";
    html << "                    const uncertainty_scale = Math.sqrt(Math.max(data.odom_only_cov_xx, data.odom_only_cov_yy)) * scale * 2;\n";
    html << "                    ctx.ellipse(centerX + data.odom_only_x * scale, centerY - data.odom_only_y * scale, uncertainty_scale, uncertainty_scale, 0, 0, 2 * Math.PI);\n";
    html << "                    ctx.stroke();\n";
    html << "                    ctx.setLineDash([]);\n";
    html << "                }\n";
    html << "                \n";
    html << "                // Ghost odometry orientation\n";
    html << "                ctx.strokeStyle = 'orange';\n";
    html << "                ctx.lineWidth = 1;\n";
    html << "                ctx.beginPath();\n";
    html << "                ctx.moveTo(centerX + data.odom_only_x * scale, centerY - data.odom_only_y * scale);\n";
    html << "                ctx.lineTo(\n";
    html << "                    centerX + data.odom_only_x * scale + Math.cos(data.odom_only_theta) * 15,\n";
    html << "                    centerY - data.odom_only_y * scale - Math.sin(data.odom_only_theta) * 15\n";
    html << "                );\n";
    html << "                ctx.stroke();\n";
    html << "                ctx.globalAlpha = 1.0;\n";
    html << "            }\n";
    html << "            \n";
    html << "            if (showGhostLidar && data.lidar_only_x !== undefined) {\n";
    html << "                // Ghost LiDAR robot (purple, semi-transparent)\n";
    html << "                ctx.globalAlpha = 0.6;\n";
    html << "                ctx.fillStyle = 'purple';\n";
    html << "                ctx.beginPath();\n";
    html << "                ctx.arc(centerX + data.lidar_only_x * scale, centerY - data.lidar_only_y * scale, 6, 0, 2 * Math.PI);\n";
    html << "                ctx.fill();\n";
    html << "                \n";
    html << "                // Ghost LiDAR uncertainty ellipse\n";
    html << "                if (data.lidar_only_cov_xx > 0 && data.lidar_only_cov_yy > 0) {\n";
    html << "                    ctx.strokeStyle = 'purple';\n";
    html << "                    ctx.lineWidth = 1;\n";
    html << "                    ctx.setLineDash([3, 3]);\n";
    html << "                    ctx.beginPath();\n";
    html << "                    const uncertainty_scale = Math.sqrt(Math.max(data.lidar_only_cov_xx, data.lidar_only_cov_yy)) * scale * 2;\n";
    html << "                    ctx.ellipse(centerX + data.lidar_only_x * scale, centerY - data.lidar_only_y * scale, uncertainty_scale, uncertainty_scale, 0, 0, 2 * Math.PI);\n";
    html << "                    ctx.stroke();\n";
    html << "                    ctx.setLineDash([]);\n";
    html << "                }\n";
    html << "                \n";
    html << "                // Ghost LiDAR orientation\n";
    html << "                ctx.strokeStyle = 'purple';\n";
    html << "                ctx.lineWidth = 1;\n";
    html << "                ctx.beginPath();\n";
    html << "                ctx.moveTo(centerX + data.lidar_only_x * scale, centerY - data.lidar_only_y * scale);\n";
    html << "                ctx.lineTo(\n";
    html << "                    centerX + data.lidar_only_x * scale + Math.cos(data.lidar_only_theta) * 15,\n";
    html << "                    centerY - data.lidar_only_y * scale - Math.sin(data.lidar_only_theta) * 15\n";
    html << "                );\n";
    html << "                ctx.stroke();\n";
    html << "                ctx.globalAlpha = 1.0;\n";
    html << "            }\n";
    html << "            \n";
    html << "            // Draw robot orientation\n";
    html << "            ctx.strokeStyle = 'black';\n";
    html << "            ctx.lineWidth = 2;\n";
    html << "            ctx.beginPath();\n";
    html << "            ctx.moveTo(centerX + data.robot_x * scale, centerY - data.robot_y * scale);\n";
    html << "            ctx.lineTo(\n";
    html << "                centerX + data.robot_x * scale + Math.cos(data.robot_theta) * 20,\n";
    html << "                centerY - data.robot_y * scale - Math.sin(data.robot_theta) * 20\n";
    html << "            );\n";
    html << "            ctx.stroke();\n";
        html << "        }\n";
        html << "        \n";
        html << "        function drawPerformanceGraph(gtsamTimes, odometryTimes, totalTimes) {\n";
        html << "            perfCtx.clearRect(0, 0, perfCanvas.width, perfCanvas.height);\n";
        html << "            \n";
        html << "            if (!gtsamTimes || gtsamTimes.length === 0) return;\n";
        html << "            \n";
        html << "            const padding = 40;\n";
        html << "            const graphWidth = perfCanvas.width - 2 * padding;\n";
        html << "            const graphHeight = perfCanvas.height - 2 * padding;\n";
        html << "            \n";
        html << "            // Find max value for scaling\n";
        html << "            const allTimes = [...gtsamTimes, ...odometryTimes, ...totalTimes];\n";
        html << "            const maxTime = Math.max(...allTimes);\n";
        html << "            const scaleY = graphHeight / (maxTime * 1.1); // Add 10% padding\n";
        html << "            \n";
        html << "            // Draw axes\n";
        html << "            perfCtx.strokeStyle = '#ccc';\n";
        html << "            perfCtx.lineWidth = 1;\n";
        html << "            perfCtx.beginPath();\n";
        html << "            perfCtx.moveTo(padding, padding);\n";
        html << "            perfCtx.lineTo(padding, perfCanvas.height - padding);\n";
        html << "            perfCtx.lineTo(perfCanvas.width - padding, perfCanvas.height - padding);\n";
        html << "            perfCtx.stroke();\n";
        html << "            \n";
        html << "            // Draw grid lines\n";
        html << "            perfCtx.strokeStyle = '#eee';\n";
        html << "            perfCtx.lineWidth = 0.5;\n";
        html << "            for (let i = 1; i <= 5; i++) {\n";
        html << "                const y = padding + (graphHeight / 5) * i;\n";
        html << "                perfCtx.beginPath();\n";
        html << "                perfCtx.moveTo(padding, y);\n";
        html << "                perfCtx.lineTo(perfCanvas.width - padding, y);\n";
        html << "                perfCtx.stroke();\n";
        html << "            }\n";
        html << "            \n";
        html << "            // Draw GTSAM times (blue line)\n";
        html << "            if (gtsamTimes.length > 1) {\n";
        html << "                perfCtx.strokeStyle = '#2196F3';\n";
        html << "                perfCtx.lineWidth = 2;\n";
        html << "                perfCtx.beginPath();\n";
        html << "                for (let i = 0; i < gtsamTimes.length; i++) {\n";
        html << "                    const x = padding + (graphWidth / (gtsamTimes.length - 1)) * i;\n";
        html << "                    const y = perfCanvas.height - padding - (gtsamTimes[i] * scaleY);\n";
        html << "                    if (i === 0) {\n";
        html << "                        perfCtx.moveTo(x, y);\n";
        html << "                    } else {\n";
        html << "                        perfCtx.lineTo(x, y);\n";
        html << "                    }\n";
        html << "                }\n";
        html << "                perfCtx.stroke();\n";
        html << "            }\n";
        html << "            \n";
        html << "            // Draw odometry times (orange line)\n";
        html << "            if (odometryTimes.length > 1) {\n";
        html << "                perfCtx.strokeStyle = '#FF9800';\n";
        html << "                perfCtx.lineWidth = 1;\n";
        html << "                perfCtx.setLineDash([2, 2]);\n";
        html << "                perfCtx.beginPath();\n";
        html << "                for (let i = 0; i < odometryTimes.length; i++) {\n";
        html << "                    const x = padding + (graphWidth / (odometryTimes.length - 1)) * i;\n";
        html << "                    const y = perfCanvas.height - padding - (odometryTimes[i] * scaleY);\n";
        html << "                    if (i === 0) {\n";
        html << "                        perfCtx.moveTo(x, y);\n";
        html << "                    } else {\n";
        html << "                        perfCtx.lineTo(x, y);\n";
        html << "                    }\n";
        html << "                }\n";
        html << "                perfCtx.stroke();\n";
        html << "                perfCtx.setLineDash([]);\n";
        html << "            }\n";
        html << "            \n";
        html << "            // Draw total times (green line)\n";
        html << "            if (totalTimes.length > 1) {\n";
        html << "                perfCtx.strokeStyle = '#4CAF50';\n";
        html << "                perfCtx.lineWidth = 1;\n";
        html << "                perfCtx.setLineDash([5, 5]);\n";
        html << "                perfCtx.beginPath();\n";
        html << "                for (let i = 0; i < totalTimes.length; i++) {\n";
        html << "                    const x = padding + (graphWidth / (totalTimes.length - 1)) * i;\n";
        html << "                    const y = perfCanvas.height - padding - (totalTimes[i] * scaleY);\n";
        html << "                    if (i === 0) {\n";
        html << "                        perfCtx.moveTo(x, y);\n";
        html << "                    } else {\n";
        html << "                        perfCtx.lineTo(x, y);\n";
        html << "                    }\n";
        html << "                }\n";
        html << "                perfCtx.stroke();\n";
        html << "                perfCtx.setLineDash([]);\n";
        html << "            }\n";
        html << "            \n";
        html << "            // Draw labels\n";
        html << "            perfCtx.fillStyle = '#333';\n";
        html << "            perfCtx.font = '12px Arial';\n";
        html << "            perfCtx.textAlign = 'center';\n";
        html << "            perfCtx.fillText('Execution Time (ms)', perfCanvas.width / 2, perfCanvas.height - 10);\n";
        html << "            \n";
        html << "            perfCtx.textAlign = 'right';\n";
        html << "            perfCtx.save();\n";
        html << "            perfCtx.translate(15, perfCanvas.height / 2);\n";
        html << "            perfCtx.rotate(-Math.PI / 2);\n";
        html << "            perfCtx.fillText('Time (ms)', 0, 0);\n";
        html << "            perfCtx.restore();\n";
        html << "            \n";
        html << "            // Draw legend\n";
        html << "            const legendY = 20;\n";
        html << "            perfCtx.textAlign = 'left';\n";
        html << "            perfCtx.font = '10px Arial';\n";
        html << "            \n";
        html << "            perfCtx.strokeStyle = '#2196F3';\n";
        html << "            perfCtx.lineWidth = 2;\n";
        html << "            perfCtx.beginPath();\n";
        html << "            perfCtx.moveTo(padding, legendY);\n";
        html << "            perfCtx.lineTo(padding + 20, legendY);\n";
        html << "            perfCtx.stroke();\n";
        html << "            perfCtx.fillText('GTSAM', padding + 25, legendY + 4);\n";
        html << "            \n";
        html << "            perfCtx.strokeStyle = '#FF9800';\n";
        html << "            perfCtx.lineWidth = 1;\n";
        html << "            perfCtx.setLineDash([2, 2]);\n";
        html << "            perfCtx.beginPath();\n";
        html << "            perfCtx.moveTo(padding + 80, legendY);\n";
        html << "            perfCtx.lineTo(padding + 100, legendY);\n";
        html << "            perfCtx.stroke();\n";
        html << "            perfCtx.setLineDash([]);\n";
        html << "            perfCtx.fillText('Odometry', padding + 105, legendY + 4);\n";
        html << "            \n";
        html << "            perfCtx.strokeStyle = '#4CAF50';\n";
        html << "            perfCtx.lineWidth = 1;\n";
        html << "            perfCtx.setLineDash([5, 5]);\n";
        html << "            perfCtx.beginPath();\n";
        html << "            perfCtx.moveTo(padding + 180, legendY);\n";
        html << "            perfCtx.lineTo(padding + 200, legendY);\n";
        html << "            perfCtx.stroke();\n";
        html << "            perfCtx.setLineDash([]);\n";
        html << "            perfCtx.fillText('Total', padding + 205, legendY + 4);\n";
        html << "        }\n";
        html << "        \n";
        html << "        function sendCommand(command) {\n";
        html << "            fetch('/api/control/' + command)\n";
        html << "                .then(response => response.text())\n";
        html << "                .then(data => console.log('Command sent:', command))\n";
        html << "                .catch(error => console.error('Error:', error));\n";
        html << "        }\n";
        html << "        \n";
        html << "        function toggleLidar() {\n";
        html << "            sendCommand('toggle_lidar');\n";
        html << "        }\n";
        html << "        \n";
        html << "        function toggleGhostOdom() {\n";
        html << "            showGhostOdom = !showGhostOdom;\n";
        html << "        }\n";
        html << "        \n";
        html << "        function toggleGhostLidar() {\n";
        html << "            showGhostLidar = !showGhostLidar;\n";
        html << "        }\n";
    html << "        \n";
        html << "        // Button state tracking\n";
        html << "        let buttonStates = {\n";
        html << "            forward: false,\n";
        html << "            backward: false,\n";
        html << "            left: false,\n";
        html << "            right: false\n";
        html << "        };\n";
        html << "        \n";
        html << "        function updateRobotMovement() {\n";
        html << "            // Update visual state of buttons\n";
        html << "            document.querySelectorAll('.control-btn').forEach(btn => {\n";
        html << "                const command = btn.getAttribute('onclick');\n";
        html << "                if (command && command.includes('sendCommand')) {\n";
        html << "                    const match = command.match(/sendCommand\\('([^']+)'\\)/);\n";
        html << "                    if (match) {\n";
        html << "                        const cmd = match[1];\n";
        html << "                        if (['forward', 'backward', 'left', 'right'].includes(cmd)) {\n";
        html << "                            if (buttonStates[cmd]) {\n";
        html << "                                btn.classList.add('pressed');\n";
        html << "                            } else {\n";
        html << "                                btn.classList.remove('pressed');\n";
        html << "                            }\n";
        html << "                        }\n";
        html << "                    }\n";
        html << "                }\n";
        html << "            });\n";
        html << "            \n";
        html << "            // Send movement command based on current button state\n";
        html << "            if (buttonStates.forward) {\n";
        html << "                sendCommand('forward');\n";
        html << "            } else if (buttonStates.backward) {\n";
        html << "                sendCommand('backward');\n";
        html << "            } else if (buttonStates.left) {\n";
        html << "                sendCommand('left');\n";
        html << "            } else if (buttonStates.right) {\n";
        html << "                sendCommand('right');\n";
        html << "            } else {\n";
        html << "                sendCommand('stop');\n";
        html << "            }\n";
        html << "        }\n";
        html << "        \n";
        html << "        // Mouse controls for buttons\n";
        html << "        document.addEventListener('mousedown', function(event) {\n";
        html << "            if (event.target.classList.contains('control-btn')) {\n";
        html << "                const command = event.target.getAttribute('onclick');\n";
        html << "                if (command && command.includes('sendCommand')) {\n";
        html << "                    const match = command.match(/sendCommand\\('([^']+)'\\)/);\n";
        html << "                    if (match) {\n";
        html << "                        const cmd = match[1];\n";
        html << "                        if (['forward', 'backward', 'left', 'right'].includes(cmd)) {\n";
        html << "                            buttonStates[cmd] = true;\n";
        html << "                            updateRobotMovement();\n";
        html << "                        }\n";
        html << "                    }\n";
        html << "                }\n";
        html << "            }\n";
        html << "        });\n";
        html << "        \n";
        html << "        document.addEventListener('mouseup', function(event) {\n";
        html << "            if (event.target.classList.contains('control-btn')) {\n";
        html << "                const command = event.target.getAttribute('onclick');\n";
        html << "                if (command && command.includes('sendCommand')) {\n";
        html << "                    const match = command.match(/sendCommand\\('([^']+)'\\)/);\n";
        html << "                    if (match) {\n";
        html << "                        const cmd = match[1];\n";
        html << "                        if (['forward', 'backward', 'left', 'right'].includes(cmd)) {\n";
        html << "                            buttonStates[cmd] = false;\n";
        html << "                            updateRobotMovement();\n";
        html << "                        }\n";
        html << "                    }\n";
        html << "                }\n";
        html << "            }\n";
        html << "        });\n";
        html << "        \n";
        html << "        // Global mouseup to release all buttons if mouse leaves the page\n";
        html << "        document.addEventListener('mouseup', function(event) {\n";
        html << "            // Release all movement buttons when mouse is released anywhere\n";
        html << "            let anyReleased = false;\n";
        html << "            if (buttonStates.forward) { buttonStates.forward = false; anyReleased = true; }\n";
        html << "            if (buttonStates.backward) { buttonStates.backward = false; anyReleased = true; }\n";
        html << "            if (buttonStates.left) { buttonStates.left = false; anyReleased = true; }\n";
        html << "            if (buttonStates.right) { buttonStates.right = false; anyReleased = true; }\n";
        html << "            if (anyReleased) updateRobotMovement();\n";
        html << "        });\n";
        html << "        \n";
        html << "        // Keyboard controls\n";
        html << "        document.addEventListener('keydown', function(event) {\n";
        html << "            switch(event.key.toLowerCase()) {\n";
        html << "                case 'w': \n";
        html << "                    if (!buttonStates.forward) {\n";
        html << "                        buttonStates.forward = true;\n";
        html << "                        updateRobotMovement();\n";
        html << "                    }\n";
        html << "                    break;\n";
        html << "                case 's': \n";
        html << "                    if (!buttonStates.backward) {\n";
        html << "                        buttonStates.backward = true;\n";
        html << "                        updateRobotMovement();\n";
        html << "                    }\n";
        html << "                    break;\n";
        html << "                case 'a': \n";
        html << "                    if (!buttonStates.left) {\n";
        html << "                        buttonStates.left = true;\n";
        html << "                        updateRobotMovement();\n";
        html << "                    }\n";
        html << "                    break;\n";
        html << "                case 'd': \n";
        html << "                    if (!buttonStates.right) {\n";
        html << "                        buttonStates.right = true;\n";
        html << "                        updateRobotMovement();\n";
        html << "                    }\n";
        html << "                    break;\n";
        html << "                case ' ': \n";
        html << "                    // Space bar stops all movement\n";
        html << "                    buttonStates.forward = false;\n";
        html << "                    buttonStates.backward = false;\n";
        html << "                    buttonStates.left = false;\n";
        html << "                    buttonStates.right = false;\n";
        html << "                    updateRobotMovement();\n";
        html << "                    break;\n";
        html << "            }\n";
        html << "        });\n";
        html << "        \n";
        html << "        document.addEventListener('keyup', function(event) {\n";
        html << "            switch(event.key.toLowerCase()) {\n";
        html << "                case 'w': \n";
        html << "                    buttonStates.forward = false;\n";
        html << "                    updateRobotMovement();\n";
        html << "                    break;\n";
        html << "                case 's': \n";
        html << "                    buttonStates.backward = false;\n";
        html << "                    updateRobotMovement();\n";
        html << "                    break;\n";
        html << "                case 'a': \n";
        html << "                    buttonStates.left = false;\n";
        html << "                    updateRobotMovement();\n";
        html << "                    break;\n";
        html << "                case 'd': \n";
        html << "                    buttonStates.right = false;\n";
        html << "                    updateRobotMovement();\n";
        html << "                    break;\n";
        html << "            }\n";
        html << "        });\n";
    html << "        \n";
        html << "        // Update data every 100ms\n";
        html << "        setInterval(updateSensorData, 100);\n";
        html << "        updateSensorData(); // Initial load\n";
        html << "        \n";
        html << "        // Update robot movement every 50ms for responsive control\n";
        html << "        setInterval(updateRobotMovement, 50);\n";
    html << "    </script>\n";
    html << "</body>\n";
    html << "</html>\n";
    
    return html.str();
}

std::string SimpleWebUI::generateJSON() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    std::ostringstream json;
    json << "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nAccess-Control-Allow-Origin: *\r\nContent-Length: ";
    
    // Helper function to convert vector to JSON array
    auto vectorToJson = [](const std::vector<double>& vec) -> std::string {
        if (vec.empty()) return "[]";
        std::ostringstream oss;
        oss << "[";
        for (size_t i = 0; i < vec.size(); ++i) {
            if (i > 0) oss << ",";
            oss << std::to_string(vec[i]);
        }
        oss << "]";
        return oss.str();
    };
    
    std::string data = "{"
         "\"left_wheel_velocity\":" + std::to_string(current_data_.left_wheel_velocity) + ","
         "\"right_wheel_velocity\":" + std::to_string(current_data_.right_wheel_velocity) + ","
         "\"lidar_x\":" + std::to_string(current_data_.lidar_x) + ","
         "\"lidar_y\":" + std::to_string(current_data_.lidar_y) + ","
         "\"lidar_theta\":" + std::to_string(current_data_.lidar_theta) + ","
         "\"lidar_confidence\":" + std::to_string(current_data_.lidar_confidence) + ","
         "\"odom_x\":" + std::to_string(current_data_.odom_x) + ","
         "\"odom_y\":" + std::to_string(current_data_.odom_y) + ","
         "\"odom_theta\":" + std::to_string(current_data_.odom_theta) + ","
         "\"robot_x\":" + std::to_string(current_data_.gtsam_x) + ","
         "\"robot_y\":" + std::to_string(current_data_.gtsam_y) + ","
         "\"robot_theta\":" + std::to_string(current_data_.gtsam_theta) + ","
         "\"gtsam_cov_xx\":" + std::to_string(current_data_.gtsam_cov_xx) + ","
         "\"gtsam_cov_yy\":" + std::to_string(current_data_.gtsam_cov_yy) + ","
         "\"gtsam_cov_tt\":" + std::to_string(current_data_.gtsam_cov_tt) + ","
         "\"lidar_enabled\":" + (current_data_.lidar_enabled ? "true" : "false") + ","
         "\"odom_only_x\":" + std::to_string(current_data_.odom_only_x) + ","
         "\"odom_only_y\":" + std::to_string(current_data_.odom_only_y) + ","
         "\"odom_only_theta\":" + std::to_string(current_data_.odom_only_theta) + ","
         "\"odom_only_cov_xx\":" + std::to_string(current_data_.odom_only_cov_xx) + ","
         "\"odom_only_cov_yy\":" + std::to_string(current_data_.odom_only_cov_yy) + ","
         "\"odom_only_cov_tt\":" + std::to_string(current_data_.odom_only_cov_tt) + ","
         "\"lidar_only_x\":" + std::to_string(current_data_.lidar_only_x) + ","
         "\"lidar_only_y\":" + std::to_string(current_data_.lidar_only_y) + ","
         "\"lidar_only_theta\":" + std::to_string(current_data_.lidar_only_theta) + ","
         "\"lidar_only_cov_xx\":" + std::to_string(current_data_.lidar_only_cov_xx) + ","
         "\"lidar_only_cov_yy\":" + std::to_string(current_data_.lidar_only_cov_yy) + ","
         "\"lidar_only_cov_tt\":" + std::to_string(current_data_.lidar_only_cov_tt) + ","
         "\"avg_gtsam_time\":" + std::to_string(current_data_.avg_gtsam_time) + ","
         "\"max_gtsam_time\":" + std::to_string(current_data_.max_gtsam_time) + ","
         "\"min_gtsam_time\":" + std::to_string(current_data_.min_gtsam_time) + ","
         "\"gtsam_times\":" + vectorToJson(current_data_.gtsam_times) + ","
         "\"odometry_times\":" + vectorToJson(current_data_.odometry_times) + ","
         "\"lidar_times\":" + vectorToJson(current_data_.lidar_times) + ","
         "\"total_times\":" + vectorToJson(current_data_.total_times) + "}";
    
    json << data.length() << "\r\n\r\n" << data;
    return json.str();
}

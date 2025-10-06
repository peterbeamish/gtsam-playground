#include "web_ui.h"
#include <iostream>
#include <sstream>
#include <iomanip>

WebUI::WebUI(int port) : port_(port), running_(false) {
    setupRoutes();
}

WebUI::~WebUI() {
    stop();
}

void WebUI::start() {
    running_ = true;
    server_thread_ = std::thread([this]() {
        app_.port(port_).multithreaded().run();
    });
    std::cout << "Web UI started on port " << port_ << std::endl;
}

void WebUI::stop() {
    running_ = false;
    if (server_thread_.joinable()) {
        server_thread_.join();
    }
}

void WebUI::updateSensorData(const SensorData& data) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_data_ = data;
}

void WebUI::setupRoutes() {
    // Main page
    CROW_ROUTE(app_, "/")([this]() {
        return crow::response(generateHTML());
    });
    
    // API endpoint for sensor data
    CROW_ROUTE(app_, "/api/sensor_data")([this]() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return crow::response(generateJSON());
    });
    
    // Control endpoint
    CROW_ROUTE(app_, "/api/control/<string>")([this](const std::string& command) {
        // This would be connected to the robot controller
        return crow::response("OK");
    });
}

std::string WebUI::generateHTML() {
    std::ostringstream html;
    html << R"(
<!DOCTYPE html>
<html>
<head>
    <title>Robot Sensor Fusion Dashboard</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background-color: #f0f0f0; }
        .container { max-width: 1200px; margin: 0 auto; }
        .header { text-align: center; margin-bottom: 30px; }
        .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; }
        .panel { background: white; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
        .sensor-data { margin: 10px 0; }
        .value { font-weight: bold; color: #333; }
        .canvas-container { text-align: center; margin-top: 20px; }
        canvas { border: 1px solid #ccc; background: white; }
        .controls { text-align: center; margin: 20px 0; }
        .control-btn { margin: 5px; padding: 10px 20px; font-size: 16px; }
        .status { text-align: center; margin: 20px 0; }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>Robot Sensor Fusion Dashboard</h1>
            <p>Real-time sensor data and GTSAM estimation</p>
        </div>
        
        <div class="controls">
            <button class="control-btn" onclick="sendCommand('forward')">W - Forward</button>
            <button class="control-btn" onclick="sendCommand('backward')">S - Backward</button>
            <button class="control-btn" onclick="sendCommand('left')">A - Left</button>
            <button class="control-btn" onclick="sendCommand('right')">D - Right</button>
            <button class="control-btn" onclick="sendCommand('stop')">Stop</button>
        </div>
        
        <div class="grid">
            <div class="panel">
                <h3>Wheel Encoders</h3>
                <div class="sensor-data">
                    <div>Left Wheel Velocity: <span class="value" id="left-wheel">0.00</span> rad/s</div>
                    <div>Right Wheel Velocity: <span class="value" id="right-wheel">0.00</span> rad/s</div>
                </div>
            </div>
            
            <div class="panel">
                <h3>LiDAR</h3>
                <div class="sensor-data">
                    <div>X: <span class="value" id="lidar-x">0.00</span> m</div>
                    <div>Y: <span class="value" id="lidar-y">0.00</span> m</div>
                    <div>θ: <span class="value" id="lidar-theta">0.00</span> rad</div>
                </div>
            </div>
            
            <div class="panel">
                <h3>Odometry</h3>
                <div class="sensor-data">
                    <div>X: <span class="value" id="odom-x">0.00</span> m</div>
                    <div>Y: <span class="value" id="odom-y">0.00</span> m</div>
                    <div>θ: <span class="value" id="odom-theta">0.00</span> rad</div>
                </div>
            </div>
            
            <div class="panel">
                <h3>GTSAM Estimate</h3>
                <div class="sensor-data">
                    <div>X: <span class="value" id="gtsam-x">0.00</span> m</div>
                    <div>Y: <span class="value" id="gtsam-y">0.00</span> m</div>
                    <div>θ: <span class="value" id="gtsam-theta">0.00</span> rad</div>
                    <div>Covariance XX: <span class="value" id="gtsam-cov-xx">0.00</span></div>
                    <div>Covariance YY: <span class="value" id="gtsam-cov-yy">0.00</span></div>
                    <div>Covariance θθ: <span class="value" id="gtsam-cov-tt">0.00</span></div>
                </div>
            </div>
        </div>
        
        <div class="canvas-container">
            <canvas id="robotCanvas" width="800" height="600"></canvas>
        </div>
        
        <div class="status">
            <p>Status: <span id="status">Connected</span></p>
        </div>
    </div>
    
    <script>
        let canvas = document.getElementById('robotCanvas');
        let ctx = canvas.getContext('2d');
        
        function updateSensorData() {
            fetch('/api/sensor_data')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('left-wheel').textContent = data.left_wheel_velocity.toFixed(2);
                    document.getElementById('right-wheel').textContent = data.right_wheel_velocity.toFixed(2);
                    document.getElementById('lidar-x').textContent = data.lidar_x.toFixed(2);
                    document.getElementById('lidar-y').textContent = data.lidar_y.toFixed(2);
                    document.getElementById('lidar-theta').textContent = data.lidar_theta.toFixed(2);
                    document.getElementById('odom-x').textContent = data.odom_x.toFixed(2);
                    document.getElementById('odom-y').textContent = data.odom_y.toFixed(2);
                    document.getElementById('odom-theta').textContent = data.odom_theta.toFixed(2);
                    document.getElementById('gtsam-x').textContent = data.gtsam_x.toFixed(2);
                    document.getElementById('gtsam-y').textContent = data.gtsam_y.toFixed(2);
                    document.getElementById('gtsam-theta').textContent = data.gtsam_theta.toFixed(2);
                    document.getElementById('gtsam-cov-xx').textContent = data.gtsam_cov_xx.toFixed(4);
                    document.getElementById('gtsam-cov-yy').textContent = data.gtsam_cov_yy.toFixed(4);
                    document.getElementById('gtsam-cov-tt').textContent = data.gtsam_cov_tt.toFixed(4);
                    
                    drawRobot(data);
                })
                .catch(error => {
                    document.getElementById('status').textContent = 'Disconnected';
                    console.error('Error:', error);
                });
        }
        
        function drawRobot(data) {
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            
            // Draw grid
            ctx.strokeStyle = '#ddd';
            ctx.lineWidth = 1;
            for (let i = 0; i < canvas.width; i += 50) {
                ctx.beginPath();
                ctx.moveTo(i, 0);
                ctx.lineTo(i, canvas.height);
                ctx.stroke();
            }
            for (let i = 0; i < canvas.height; i += 50) {
                ctx.beginPath();
                ctx.moveTo(0, i);
                ctx.lineTo(canvas.width, i);
                ctx.stroke();
            }
            
            // Scale factor for visualization
            const scale = 50; // pixels per meter
            const centerX = canvas.width / 2;
            const centerY = canvas.height / 2;
            
            // Draw odometry position
            ctx.fillStyle = 'blue';
            ctx.beginPath();
            ctx.arc(centerX + data.odom_x * scale, centerY - data.odom_y * scale, 5, 0, 2 * Math.PI);
            ctx.fill();
            
            // Draw GTSAM estimate
            ctx.fillStyle = 'red';
            ctx.beginPath();
            ctx.arc(centerX + data.gtsam_x * scale, centerY - data.gtsam_y * scale, 8, 0, 2 * Math.PI);
            ctx.fill();
            
            // Draw LiDAR position
            ctx.fillStyle = 'green';
            ctx.beginPath();
            ctx.arc(centerX + data.lidar_x * scale, centerY - data.lidar_y * scale, 3, 0, 2 * Math.PI);
            ctx.fill();
            
            // Draw robot orientation
            ctx.strokeStyle = 'black';
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(centerX + data.gtsam_x * scale, centerY - data.gtsam_y * scale);
            ctx.lineTo(
                centerX + data.gtsam_x * scale + Math.cos(data.gtsam_theta) * 20,
                centerY - data.gtsam_y * scale - Math.sin(data.gtsam_theta) * 20
            );
            ctx.stroke();
        }
        
        function sendCommand(command) {
            fetch('/api/control/' + command)
                .then(response => response.text())
                .then(data => console.log('Command sent:', command))
                .catch(error => console.error('Error:', error));
        }
        
        // Keyboard controls
        document.addEventListener('keydown', function(event) {
            switch(event.key.toLowerCase()) {
                case 'w': sendCommand('forward'); break;
                case 's': sendCommand('backward'); break;
                case 'a': sendCommand('left'); break;
                case 'd': sendCommand('right'); break;
                case ' ': sendCommand('stop'); break;
            }
        });
        
        // Update data every 100ms
        setInterval(updateSensorData, 100);
        updateSensorData(); // Initial load
    </script>
</body>
</html>
    )";
    return html.str();
}

std::string WebUI::generateJSON() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    std::ostringstream json;
    json << "{"
         << "\"left_wheel_velocity\":" << current_data_.left_wheel_velocity << ","
         << "\"right_wheel_velocity\":" << current_data_.right_wheel_velocity << ","
         << "\"lidar_x\":" << current_data_.lidar_x << ","
         << "\"lidar_y\":" << current_data_.lidar_y << ","
         << "\"lidar_theta\":" << current_data_.lidar_theta << ","
         << "\"odom_x\":" << current_data_.odom_x << ","
         << "\"odom_y\":" << current_data_.odom_y << ","
         << "\"odom_theta\":" << current_data_.odom_theta << ","
         << "\"gtsam_x\":" << current_data_.gtsam_x << ","
         << "\"gtsam_y\":" << current_data_.gtsam_y << ","
         << "\"gtsam_theta\":" << current_data_.gtsam_theta << ","
         << "\"gtsam_cov_xx\":" << current_data_.gtsam_cov_xx << ","
         << "\"gtsam_cov_yy\":" << current_data_.gtsam_cov_yy << ","
         << "\"gtsam_cov_tt\":" << current_data_.gtsam_cov_tt
         << "}";
    return json.str();
}

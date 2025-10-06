cc_library(
    name = "sensors",
    srcs = ["sensor_simulator.cpp"],
    hdrs = ["sensor_simulator.h"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "odometry",
    srcs = ["odometry_processor.cpp"],
    hdrs = ["odometry_processor.h"],
    deps = ["//src/sensors:sensors"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "gtsam_integration",
    srcs = ["gtsam_integrator.cpp"],
    hdrs = ["gtsam_integrator.h"],
    deps = [
        "//src/odometry:odometry",
        "//:gtsam_system",
        "//:eigen_system",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "robot",
    srcs = ["robot_controller.cpp"],
    hdrs = ["robot_controller.h"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "web_ui",
    srcs = ["web_ui.cpp"],
    hdrs = ["web_ui.h"],
    deps = ["//:boost_system"],
    visibility = ["//visibility:public"],
)

# Main robot simulation with web interface and GTSAM
cc_binary(
    name = "robot_simulation",
    srcs = ["main.cpp"],
    copts = ["-std=c++11"],
    deps = [
        "//src/sensors:sensors",
        "//src/odometry:odometry",
        "//src/gtsam_integration:gtsam_integration",
        "//src/robot:robot",
        "//src/web_ui:simple_web_ui",
        "//src/performance:performance_monitor",
    ],
)

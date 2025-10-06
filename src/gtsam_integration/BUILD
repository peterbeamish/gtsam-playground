cc_library(
    name = "gtsam_integration",
    srcs = ["gtsam_integrator.cpp"],
    hdrs = ["gtsam_integrator.h"],
    includes = ["/usr/include/eigen3"],
    deps = [
        "//src/odometry:odometry",
        "//:gtsam_system",
        "//:eigen_system",
    ],
    visibility = ["//visibility:public"],
)

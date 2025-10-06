cc_library(
    name = "gtsam_integration",
    srcs = ["gtsam_integrator.cpp"],
    hdrs = ["gtsam_integrator.h"],
    deps = [
        "//src/odometry:odometry",
        "@gtsam//:gtsam",
        "@eigen//:eigen",
    ],
    visibility = ["//visibility:public"],
)

# Bazel configuration for system-installed GTSAM
cc_library(
    name = "gtsam_system",
    includes = ["/usr/local/include", "/usr/include/eigen3"],
    linkopts = [
        "-L/usr/local/lib",
        "-lgtsam",
        "-lgtsam_unstable",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "eigen_system",
    includes = ["/usr/include/eigen3"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "boost_system",
    includes = ["/usr/include"],
    linkopts = [
        "-lboost_system",
        "-lboost_thread", 
        "-lboost_chrono",
        "-lboost_date_time",
        "-lboost_regex",
        "-lboost_program_options",
        "-lboost_timer",
        "-lboost_filesystem",
    ],
    visibility = ["//visibility:public"],
)

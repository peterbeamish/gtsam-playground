workspace(name = "gtsam_playground")

# Define system Eigen using new_local_repository
new_local_repository(
    name = "eigen",
    path = "/usr/include/eigen3",
    build_file_content = """
cc_library(
    name = "eigen",
    hdrs = glob(["Eigen/**/*"]),
    includes = ["."],
    visibility = ["//visibility:public"],
)
""",
)

# Define system GTSAM using new_local_repository
new_local_repository(
    name = "gtsam",
    path = "/usr/local",
    build_file_content = """
cc_library(
    name = "gtsam",
    hdrs = glob(["include/gtsam/**/*"]),
    includes = ["include"],
    linkopts = [
        "/usr/local/lib/libgtsam.a",
        "/usr/local/lib/libgtsam_unstable.a",
        "/usr/local/lib/libmetis-gtsam.a",
        "-lboost_timer",
        "-lboost_serialization",
        "-lboost_system",
        "-lboost_thread",
        "-lboost_chrono",
        "-lboost_date_time",
        "-lboost_regex",
        "-lboost_program_options",
        "-lboost_filesystem",
    ],
    visibility = ["//visibility:public"],
    deps = ["@eigen//:eigen"],
)
""",
)

# Define system Boost using new_local_repository
new_local_repository(
    name = "boost",
    path = "/usr",
    build_file_content = """
cc_library(
    name = "boost",
    hdrs = glob(["include/boost/**/*"]),
    includes = ["include"],
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
""",
)

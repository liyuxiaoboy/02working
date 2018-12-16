load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "poses_interpolation_lib",
    srcs = [
        "poses_interpolation.cc",
        "vehicle_dynamic_model.cc",
    ],
    hdrs = [
        "poses_interpolation.h",
        "vehicle_dynamic_model.h",
    ],
    linkopts = [
        "-lboost_filesystem",
    ],
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/control/common:interpolation_2d",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/proto:sim_control_conf_proto",
        "//modules/localization/msf/common/io:localization_msf_common_io",
        "//modules/localization/msf/local_map/base_map:localization_msf_base_map",
        "@com_google_gflags//:gflags",
        "@eigen//:eigen",
    ],
)

cpplint()

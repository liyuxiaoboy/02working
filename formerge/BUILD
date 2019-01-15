load("//tools:cpplint.bzl", "cpplint")
load("//tools:moonx.bzl", "mx_cc_library", "mx_cc_binary")

package(default_visibility = ["//visibility:public"])

mx_cc_binary(
    name = "bag2image",
    srcs = [
        "bag2image.cc",
    ],
    deps = [
        "//modules/perception/traffic_light/util",
        "@com_google_gflags//:gflags",
        "@opencv2//:core",
        "@opencv2//:highgui",
        "@ros//:ros_common",
    ],
)

mx_cc_binary(
    name = "colored_pointcloud",
    srcs = [
        "colored_pointcloud.cc",
    ],
    copts = [
        "-Wno-deprecated",
    ],
    linkopts = [
        "-lboost_filesystem",
    ],
    deps = [
        "//modules/localization/msf/common/io:localization_msf_common_io",
        "@com_google_gflags//:gflags",
        "@eigen//:eigen",
        "@opencv2//:calib3d",
        "@opencv2//:core",
        "@opencv2//:highgui",
        "@pcl//:pcl",
        "@vtk",
        "@yaml_cpp//:yaml",
    ],
)

mx_cc_binary(
    name = "merge_pointclouds",
    srcs = [
        "merge_pointclouds.cc",
    ],
    copts = [
        "-Wno-deprecated",
    ],
    linkopts = [
        "-lboost_filesystem",
    ],
    deps = [
        "//modules/localization/msf/common/io:localization_msf_common_io",
        "@com_google_gflags//:gflags",
        "@eigen//:eigen",
        "@pcl//:pcl",
        "@vtk",
    ],
)


mx_cc_binary(
    name = "pcd_merge",
    srcs = [
        "bazel_pcd_merge.cc",
    ],
    copts = [
        "-Wno-deprecated",
    ],
    linkopts = [
        "-lboost_filesystem",
    ],
    deps = [
        "//modules/localization/msf/common/io:localization_msf_common_io",
        "@com_google_gflags//:gflags",
        "@eigen//:eigen",
        "@pcl//:pcl",
        "@vtk",
    ],
)

mx_cc_binary(
    name = "colored_pointcloud_no_obs",
    srcs = [
        "bazel_color_point_no_obs.cc",
    ],
    copts = [
        "-Wno-deprecated",
    ],
    linkopts = [
        "-lboost_filesystem",
    ],
    deps = [
        "//modules/localization/msf/common/io:localization_msf_common_io",
        "@com_google_gflags//:gflags",
        "@eigen//:eigen",
        "@opencv2//:calib3d",
        "@opencv2//:core",
        "@opencv2//:highgui",
        "@pcl//:pcl",
        "@vtk",
        "@yaml_cpp//:yaml",
    ],
)
cpplint()

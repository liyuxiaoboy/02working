#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include <boost/filesystem.hpp>

#include "colored_pointcloud.h"
#include "readKitti.h"

// inline Eigen::Matrix4d transInverse(const Eigen::Matrix4d& mat_in)
// {
//     Eigen::Affine3d input;
//     input.linear() = mat_in.block(0, 0, 3, 3);
//     input.translation() = mat_in.block(0, 3, 3, 1);
//     return input.inverse().matrix();
// }

inline Eigen::Affine3d toAffine(const Eigen::Matrix4d &mat_in)
{
    Eigen::Affine3d input;
    input.linear() = mat_in.block(0, 0, 3, 3);
    input.translation() = mat_in.block(0, 3, 3, 1);
    return input;
}

int main(int argc, char **argv)
{
    if (argc != 8)
    {
        std::cout << "please input with format: ./color_1 [pcd_file] [output_pcd_dir/]"
                     "[cam] [imu] [velo] [poses] [num]\n";
        return 0;
    }
    std::string pcd_base = argv[1];
    std::string output_path = argv[2];
    std::string cam_to_cam = argv[3];
    std::string imu_to_velo = argv[4];
    std::string velo_to_cam = argv[5];
    std::string poses_file = argv[6];
    int num = std::stoi(argv[7]);

    Calib calibration;
    calibration.readKittiCalibCamToCam(cam_to_cam);
    calibration.readKittiCalibImuToVelo(imu_to_velo);
    calibration.readKittiCalibVeloToCam(velo_to_cam);
    // std::cout << "111\n";
    std::vector<Eigen::Matrix4d> poses;
    readKittiPose(poses_file, poses);

    // std::cout << poses[3];

    // Eigen::Matrix4d R_rect4d_0 = transTo4d(calibration.cam[0].R_rect);
    Eigen::Matrix4d R_rect4d_0 = Eigen::Matrix4d::Identity();
    R_rect4d_0.block(0, 0, 3, 3) = calibration.cam[0].R_rect;

    // Eigen::Matrix4d P_rect4d_0 = transTo4d(calibration.cam[0].P_rect);
    Eigen::Matrix4d P_rect4d_0 = Eigen::Matrix4d::Identity();
    P_rect4d_0.block(0, 0, 3, 4) = calibration.cam[0].P_rect;
    // std::cout << "P_rect: \n"
    //                   << P_rect4d_0 << "\n";
    //四元数
    Eigen::Affine3d P_velo_to_image_0 = toAffine(
        R_rect4d_0 *
        calibration.velo_to_cam);
    // std::cout << "P_velo_to_image_0: \n"
    //                   << P_velo_to_image_0.matrix() << "\n";

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_base(new pcl::PointCloud<pcl::PointXYZRGB>);
   // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (int i = 0; i < num; i++)
    {
        char name[100];
        sprintf(name, "%06d", i);

        std::string path_tmp = pcd_base + name + ".pcd";
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::io::loadPCDFile(path_tmp, *cloud_rgb);

        if (i == 0)
        {
            *cloud_base = *cloud_rgb;
        }
        else
        {
            Eigen::Affine3d trans, pose;
            pose = toAffine(poses[i]);
            trans = P_velo_to_image_0.inverse() * pose * P_velo_to_image_0;

            // std::cout << "trans: \n"
            //           << trans.matrix() << "\n";
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tmp_out(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::transformPointCloud(*cloud_rgb, *cloud_tmp_out, trans);

            *cloud_base += *cloud_tmp_out;
           // *cloud_1 = *cloud_rgb;
        }
    }

    pcl::io::savePCDFileASCII(output_path + "sum.pcd", *cloud_base);
    std::cout << "Saved " << cloud_base->points.size() << " data points to " << output_path + "sum.pcd" << std::endl;

    pcl::visualization::PCLVisualizer viewer("Cloud viewer");
    viewer.addPointCloud(cloud_base, " cloud final");
    // viewer.addPointCloud(cloud_1, " cloud final 1");
    viewer.setBackgroundColor(0, 0, 0);
    viewer.addCoordinateSystem(10.0);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
    }

    return 0;
}
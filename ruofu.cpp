#include "opencv2/opencv.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <boost/filesystem.hpp>
#include <Eigen>

//外参矩阵输入
Eigen::Affine3d extrinsic_l_c;
extrinsic_l_c.translation() = Eigen::Vector3d(0.378653, -0.14088, -0.567647);
extrinsic_l_c.linear() = q_c_l.matrix();
//在这里你直接对外参求逆
extrinsic_l_c = extrinsic_l_c.inverse();

//这是求的转移矩阵
Eigen::Matrix4d trans_pcd_img;
Eigen::Affine3d affine_p_i;

getTransform(pcd_iter->first, img_time, affine_p_i, pcd_poses, img_time_poses);
{
    void getTransform(long idx_1, long idx_2, Eigen::Affine3d &T, std::map<long, double[7]> &poses_map, std::map<long, double[7]> &img_time_poses_map)
{
    Eigen::Quaterniond q_1(poses_map[idx_1][6], poses_map[idx_1][3], poses_map[idx_1][4], poses_map[idx_1][5]);
    Eigen::Quaterniond q_2(img_time_poses_map[idx_2][6], img_time_poses_map[idx_2][3], img_time_poses_map[idx_2][4], img_time_poses_map[idx_2][5]);

    Eigen::Affine3d p_1, p_2;
    p_1.translation() = Eigen::Vector3d(poses_map[idx_1][0], poses_map[idx_1][1], poses_map[idx_1][2]);
    p_1.linear() = q_1.matrix();
    p_2.translation() = Eigen::Vector3d(img_time_poses_map[idx_2][0], img_time_poses_map[idx_2][1], img_time_poses_map[idx_2][2]);
    p_2.linear() = q_2.matrix();
    //看这里 p_1是q_1=poses_map p_2是q_2=img_time_poses_map
    //在这里用了p_2的逆乘p_1得到T 也就是说 p1=p2*T
    T = p_2.inverse() * p_1;
}
}
//在这里用extrinsic_l_c * affine_p_i也就是一个取了inverse的外参乘了T
Eigen::Affine3d affine_l_c = extrinsic_l_c * affine_p_i;
trans_pcd_img = affine_l_c.matrix();
//然后就把这个矩阵传给pcl的投影矩阵了
//cloud_src是pcd的点云
pcl::transformPointCloud(*cloud_src, *cloud_cam, trans_pcd_img);

//trans_pcd_img=extrinsic_l_c * affine_p_i
            =extrinsic_l_c*T
            =extrinsic_l_c*p_2.inverse() * p_1
            =extrinsic_l_c*img_time_poses_map.inverse()*poses_map
            //这里的extrinsic_l_c是个求逆的结果
            //转移矩阵其实长这样。。。
            =extrinsic_l_c.inverse()*img_time_poses_map.inverse()*poses_map
            //用上面那个去乘pcd的位资也就是poses_map
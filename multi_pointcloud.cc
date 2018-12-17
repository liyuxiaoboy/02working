#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>

inline std::string getPcdFile(const std::string &path, int num)
{
    return path + std::to_string(num) + ".pcd";
}

int getNum(const std::string &path)
{
    int num = 0;
    int j = 1;
    for (int i = 5;; i++)
    {
        char tmp = path[path.length() - i];
        // std::cout<<tmp<<" ";
        if (tmp < '0' || tmp > '9')
            break;
        num += (tmp - '0') * j;
        j = j * 10;
    }
    return num;
}

void getPose(const std::string &path, std::map<int, double[7]> &map)
{
    std::ifstream fin(path, std::ios::in);
    char line[1024] = {0};
    std::string name = "";
    std::string timestamp = "";
    double pose[7];
    while (fin.getline(line, sizeof(line)))
    {
        std::stringstream word(line);
        word >> name;
        word >> timestamp;
        for (int i = 0; i < 7; i++)
        {
            std::string tmp;
            word >> tmp;
            pose[i] = stod(tmp);
        }
        for (int i = 0; i < 7; i++)
        {
            map[getNum(name)][i] = pose[i];
        }
        // printf("%lf %lf %lf %lf %lf %lf %lf\n", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6]);
        // std::cout<<num<<" "<<pose[0]<<" "<<pose[1]<<" "<<pose[2]<<" "<<pose[3]<<" "<<pose[4]<<" "<<pose[5]<<" "<<pose[6]<<"\n";
    }
    fin.clear();
    fin.close();
}

//get tranform from index_1 to index_2
void getTransform(int idx_1, int idx_2, Eigen::Matrix4d &T, std::map<int, double[7]> &poses_map)
{
    Eigen::Quaterniond q_1(poses_map[idx_1][6], poses_map[idx_1][3], poses_map[idx_1][4], poses_map[idx_1][5]);
    Eigen::Quaterniond q_2(poses_map[idx_2][6], poses_map[idx_2][3], poses_map[idx_2][4], poses_map[idx_2][5]);

    Eigen::Affine3d p_1, p_2;
    p_1.translation() = Eigen::Vector3d(poses_map[idx_1][0], poses_map[idx_1][1], poses_map[idx_1][2]);
    p_1.linear() = q_1.matrix();
    p_2.translation() = Eigen::Vector3d(poses_map[idx_2][0], poses_map[idx_2][1], poses_map[idx_2][2]);
    p_2.linear() = q_2.matrix();
    auto mat = p_2.inverse() * p_1;
    // std::cout << mat.matrix() << "\n";
    T = mat.matrix();
}

int main(int argc, char **argv)
{
    if (argc != 6)
    {
        std::cout << "please input with the format: ./multi_pointcloud [pcd_files_dir/]
         [pcd_poses.txt] [number of begin pcd] [number of pcd to be registration, 0 means all] [output pcd's path]" << std::endl;
        return 0;
    }
    std::string pcd_dir = argv[1];
    std::string odom_path = argv[2];
    int begin = std::stoi(argv[3]);
    int sum = std::stoi(argv[4]);
    std::string output_path = argv[5];

    std::map<int, double[7]> poses_map;
    getPose(odom_path, poses_map);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sum(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_base(new pcl::PointCloud<pcl::PointXYZRGB>);

    // if (begin == 0)
    //     auto iter=pose_map.begin();
    // else

    int base_index = 0;
    for (auto iter = poses_map.begin(); iter != poses_map.end(); iter++)
    {
        std::string path_tmp = pcd_dir + std::to_string(iter->first) + ".pcd";
        ifstream fin(path_tmp);
        if (!fin)
        {
            // std::cout<<iter->first<<"\n";
            continue;
        }
        if (base_index == 0)
        {
            base_index = iter->first;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::io::loadPCDFile(path_tmp, *cloud_tmp);
            *cloud_sum = *cloud_tmp;
            // std::cout<<"base\n";
        }
        else
        {
            Eigen::Matrix4d trans;
            getTransform(iter->first, base_index, trans, poses_map);

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::io::loadPCDFile(path_tmp, *cloud_tmp);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tmp_out(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::transformPointCloud(*cloud_tmp, *cloud_tmp_out, trans);

            *cloud_sum += *cloud_tmp_out;
            // std::cout<<iter->first<<"\n";
        }
    }

    pcl::io::savePCDFileASCII(output_path, *cloud_sum);
    std::cout << "Saved " << cloud_sum->points.size() << " data points to " << output_path << std::endl;

    pcl::visualization::PCLVisualizer viewer("Cloud viewer");
    viewer.addPointCloud(cloud_sum, " cloud final");
    viewer.setBackgroundColor(0, 0, 0);
    viewer.addCoordinateSystem(10.0);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
    }

    // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // icp.setInputSource(cloud_in);
    // icp.setInputTarget(cloud_out);
    // //Set the max correspondence distance to 4cm (e.g., correspondences with higher distances will be ignored)
    // icp.setMaxCorrespondenceDistance(0.1);
    // // 最大迭代次数
    // icp.setMaximumIterations(5000);
    // // 两次变化矩阵之间的差值
    // icp.setTransformationEpsilon(1e-10);
    // // 均方误差
    // icp.setEuclideanFitnessEpsilon(0.2);
    // pcl::PointCloud<pcl::PointXYZ> Final;
    // icp.align(Final, );
    // std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
    // std::cout << icp.getFinalTransformation() << std::endl;

    return (0);
}
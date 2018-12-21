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
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>

int getNum(const std::string &path)
{
    int num = 0;
    int j = 1;
    for (int i = 5;; i++)
    {
        char tmp = path[path.length() - i];
        if (tmp =='/')
            break;
        num += (tmp - '0') * j;
        j = j * 10;
    }
    std::cout <<num<<"\n";
    return num;
}

void getPcdFilesAndPoses(const std::string &path, std::map<long, std::string> &files, std::map<long, double[7]> &poses)
{
    std::ifstream fin(path, std::ios::in);
    char line[1024] = {0};
    std::string name = "";
    std::string timestamp = "";
    // double pose[7];
    while (fin.getline(line, sizeof(line)))
    {
        std::stringstream word(line);
        word >> name;
        word >> timestamp;
        long pcd_time = stod(timestamp) * 1e6;
        files[pcd_time] ="/home/yxli" + name;
        for (int i = 0; i < 7; i++)
        {
            std::string tmp;
            word >> tmp;
            poses[pcd_time][i] = stod(tmp);
        }

    }
    fin.clear();
    fin.close();
}


//get tranform from pcd time to image time
void getTransform(long idx_1, long idx_2, Eigen::Affine3d &T, std::map<long, double[7]> &poses_map, std::map<long, double[7]> &img_time_poses_map)
{
    Eigen::Quaterniond q_1(poses_map[idx_1][3], poses_map[idx_1][4], poses_map[idx_1][5], poses_map[idx_1][6]);
    Eigen::Quaterniond q_2(img_time_poses_map[idx_2][3], img_time_poses_map[idx_2][4], img_time_poses_map[idx_2][5], img_time_poses_map[idx_2][6]);

    Eigen::Affine3d p_1, p_2;
    p_1.translation() = Eigen::Vector3d(poses_map[idx_1][0], poses_map[idx_1][1], poses_map[idx_1][2]);
    p_1.linear() = q_1.matrix();
    p_2.translation() = Eigen::Vector3d(img_time_poses_map[idx_2][0], img_time_poses_map[idx_2][1], img_time_poses_map[idx_2][2]);
    p_2.linear() = q_2.matrix();
    T = p_2.inverse() * p_1;

}



int main(int argc, char **argv)
{
    if (argc != 3)
    {
        std::cout << "please input with format: ./pcd_merge [pcd_poses_path] [opt_path] \n";
        return 0;
    }
//pcd_pose
    std::string pcd_path = argv[1];
    std::string output_pcd_path = argv[2];

    std::map<long, std::string> pcd_map;
    std::map<long, double[7]> pcd_poses;
//read pcd_pose.txt   pcd_poses[pcd_time][7] pcd_map[pcd_time][s]
    getPcdFilesAndPoses(pcd_path, pcd_map, pcd_poses);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sum(new pcl::PointCloud<pcl::PointXYZ>);
    long base_index = 0;
    int count=0;
    for ( auto pcd_iter = pcd_map.begin() ; pcd_iter != pcd_map.end(); pcd_iter++)
    {
            if(base_index==0)
            {
                base_index=pcd_iter->first;
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::io::loadPCDFile(pcd_iter->second, *cloud_temp);
                *cloud_sum = *cloud_temp;
            }
            else
            {
                Eigen::Matrix4d trans_temp_base;
                Eigen::Affine3d affine_p_i;
                getTransform(pcd_iter->first, base_index, affine_p_i, pcd_poses, pcd_poses);
                trans_temp_base = affine_p_i.matrix();
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_j(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::io::loadPCDFile(pcd_iter->second, *cloud_j);
                pcl::transformPointCloud(*cloud_j, *cloud_tmp, trans_temp_base);
                *cloud_sum += *cloud_tmp;
            }
    
    std::cout<<"merge the "<<count<<" number. "<<endl;
    count++;
    }
    pcl::io::savePCDFileBinary(output_pcd_path , *cloud_sum);
    

    return 0;
}
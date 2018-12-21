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

void getImgFilesAndPOses(const std::string &image_path, std::map<long, std::string> &files, std::map<long, double[7]> &poses)
{
    std::ifstream fin(image_path, std::ios::in);
    char line[1024] = {0};
    std::string name = "";
    std::string timestamp = "";
    while (fin.getline(line, sizeof(line)))
    {
        std::stringstream word(line);
        word >> name;
        name = std::to_string(getNum(name));
        word >> timestamp;
        long img_time = stod(timestamp) * 1e6;
        files[img_time] ="/home/yxli/apollo/modules/perception/data/yolo_camera_detector_test/opt/" + name +".png";
        //std::cout <<files[img_time]  <<" is ok "<<"\n";

        for (int i = 0; i < 7; i++)
        {
            std::string tmp;
            word >> tmp;
            poses[img_time][i] = stod(tmp);
        }
    }
    fin.clear();
    fin.close();
}

bool getClosestImage(long pcd_time, const std::map<long, std::string> &img_map,
                     std::map<long, std::string>::iterator img_iter, long &img_time)
{
    if (img_iter == img_map.end() || pcd_time < img_iter->first)
        return false;

    if (pcd_time == img_iter->first)
    {
        img_time = img_iter->first;
        return true;
    }

    auto img_next = img_iter;
    img_next++;
    while (pcd_time > img_iter->first && img_next != img_map.end() && pcd_time > img_next->first)
    {
        img_iter++;
        img_next++;
    }
    if (pcd_time > img_iter->first && pcd_time < img_next->first)
    {
        img_time = (((pcd_time - img_iter->first) - (img_next->first - pcd_time)) > 0) ? img_next->first : img_iter->first;
        return true;
    }
    return false;
}

std::vector<cv::Point3d> get3DPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    std::vector<cv::Point3d> points;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, *cloud_f);
    for (int i = 0; i < cloud->points.size(); i++)
    {
        points.push_back(cv::Point3d(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z));
    }

    return points;
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

std::vector<int> GetObstacleID(const std::string &path)
{
    std::ifstream fin(path, std::ios::in);
    char line[1024] = {0};
    std::vector<int> obstacle_id;
    while (fin.getline(line, sizeof(line)))
    {
        std::string idx;
        std::stringstream word(line);
        word >> idx;
        obstacle_id.push_back(std::stoi(idx));
    }
    fin.clear();
    fin.close();
    return obstacle_id;
}

int main(int argc, char **argv)
{
    if (argc != 6)
    {
        std::cout << "please input with format: ./color_1 [pcd_poses_path] [img_poses_path]  [output_pcd_dir/] [output_image_dir/] [range limitation, 0 means no limitation]\n";
        return 0;
    }
//pcd_pose
    std::string pcd_path = argv[1];
//可以去了多余输入
    std::string img_path = argv[2];
//image_pose.txt
    std::string output_pcd_path = argv[3];
    std::string output_img_path = argv[4];
    double range = std::stod(argv[5]);


    std::map<long, std::string> pcd_map;
    std::map<long, std::string> img_map;
    std::map<long, double[7]> pcd_poses;
    std::map<long, double[7]> img_time_poses;
//read pcd_pose.txt image_pose.txt
    getPcdFilesAndPoses(pcd_path, pcd_map, pcd_poses);
    getImgFilesAndPOses(img_path, img_map, img_time_poses);

//ext transform between lidar and camera
    Eigen::Matrix4d trans_l_c, trans_c_l;
    Eigen::Quaterniond q_c_l(0.497558, -0.433905, 0.495593, -0.564402);
    Eigen::Affine3d extrinsic_l_c;
    extrinsic_l_c.translation() = Eigen::Vector3d(0.378653, -0.14088, -0.567647);
    extrinsic_l_c.linear() = q_c_l.matrix();
    extrinsic_l_c = extrinsic_l_c.inverse();
    trans_l_c = extrinsic_l_c.matrix();
    trans_c_l = extrinsic_l_c.inverse().matrix();
    // Eigen::Matrix4d trans_l_c, trans_c_l;
    // Eigen::Quaterniond q_c_l(0.538867, -0.480859, 0.460358, -0.516205);
    // Eigen::Affine3d extrinsic_l_c;
    // extrinsic_l_c.translation() = Eigen::Vector3d(0.546826, 0.053769, 0.356172);
    // extrinsic_l_c.linear() = q_c_l.matrix();
    // extrinsic_l_c = extrinsic_l_c.inverse();
    // trans_l_c = extrinsic_l_c.matrix();
    // trans_c_l = extrinsic_l_c.inverse().matrix();


//intrisic transform of iamge
    std::vector<cv::Point2d> imagePoints;
    // Intrisic matrix
    cv::Mat intrisicMat(3, 3, cv::DataType<double>::type);
    intrisicMat.at<double>(0, 0) = 1978.606715;
    intrisicMat.at<double>(1, 0) = 0;
    intrisicMat.at<double>(2, 0) = 0;

    intrisicMat.at<double>(0, 1) = 0;
    intrisicMat.at<double>(1, 1) = 1972.693783;
    intrisicMat.at<double>(2, 1) = 0;

    intrisicMat.at<double>(0, 2) = 916.698435;
    intrisicMat.at<double>(1, 2) = 534.97131;
    intrisicMat.at<double>(2, 2) = 1;

    cv::Mat rVec(3, 1, cv::DataType<double>::type); // Rotation vector
    rVec.at<double>(0) = 0;
    rVec.at<double>(1) = 0;
    rVec.at<double>(2) = 0;

    cv::Mat tVec(3, 1, cv::DataType<double>::type); // Translation vector
    tVec.at<double>(0) = 0;
    tVec.at<double>(1) = 0;
    tVec.at<double>(2) = 0;

    cv::Mat distCoeffs(5, 1, cv::DataType<double>::type); // Distortion vector
    distCoeffs.at<double>(0) = -0.546301;
    distCoeffs.at<double>(1) = 0.273115;
    distCoeffs.at<double>(2) = 0.005977;
    distCoeffs.at<double>(3) = 0.002924;
    distCoeffs.at<double>(4) = 0;

// //intrisic transform of iamge
//     std::vector<cv::Point2d> imagePoints;
//     // Intrisic matrix
//     cv::Mat intrisicMat(3, 3, cv::DataType<double>::type);
//     intrisicMat.at<double>(0, 0) = 1984.964717;
//     intrisicMat.at<double>(1, 0) = 0;
//     intrisicMat.at<double>(2, 0) = 0;

//     intrisicMat.at<double>(0, 1) = 0;
//     intrisicMat.at<double>(1, 1) = 1996.986882;
//     intrisicMat.at<double>(2, 1) = 0;

//     intrisicMat.at<double>(0, 2) = 842.779003;
//     intrisicMat.at<double>(1, 2) = 593.359923;
//     intrisicMat.at<double>(2, 2) = 1;

//     cv::Mat rVec(3, 1, cv::DataType<double>::type); // Rotation vector
//     rVec.at<double>(0) = 0;
//     rVec.at<double>(1) = 0;
//     rVec.at<double>(2) = 0;

//     cv::Mat tVec(3, 1, cv::DataType<double>::type); // Translation vector
//     tVec.at<double>(0) = 0;
//     tVec.at<double>(1) = 0;
//     tVec.at<double>(2) = 0;

//     cv::Mat distCoeffs(5, 1, cv::DataType<double>::type); // Distortion vector
//     distCoeffs.at<double>(0) = -0.542865;
//     distCoeffs.at<double>(1) = 0.245749;
//     distCoeffs.at<double>(2) = 0.005958;
//     distCoeffs.at<double>(3) = 0.005136;
//     distCoeffs.at<double>(4) = 0;
//遮挡点标记
//遮挡点标记
    int flag = 0;

    cv::Mat image;

    auto img_iter = img_map.begin();
    for (auto pcd_iter = pcd_map.begin(); pcd_iter != pcd_map.end(); pcd_iter++)
    {
        long img_time;
        //matching image and pcd
        if (!getClosestImage(pcd_iter->first, img_map, img_iter, img_time))
            continue;

        cv::Mat img;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZ>);
        //load pcd
        if(pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_iter->second, *cloud_src)==-1)
        {
            continue;
        }

        //todo transform from pcd's time to img's time
        Eigen::Matrix4d trans_pcd_img;
        Eigen::Matrix4d trans_img_pcd;
        Eigen::Affine3d affine_p_i;
        getTransform(pcd_iter->first, img_time, affine_p_i, pcd_poses, img_time_poses);
        Eigen::Affine3d affine_l_c = extrinsic_l_c * affine_p_i;


        trans_pcd_img = affine_l_c.matrix();
        trans_img_pcd = affine_l_c.inverse().matrix();


        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cam(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud_src, *cloud_cam, trans_pcd_img);

        std::vector<cv::Point3d> objectPoints = get3DPoints(cloud_cam);

        cv::projectPoints(objectPoints, rVec, tVec, intrisicMat, distCoeffs, imagePoints);

        image = cv::imread(img_map[img_time], CV_LOAD_IMAGE_COLOR);

        cv::Mat depth_2 = cv::Mat::zeros(image.cols, image.rows, CV_32FC1);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cam_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);

        std::cout << pcd_iter->first <<" "<<img_time<<"\n";

        for (int i = 0; i < imagePoints.size(); i++)
        {
            auto col = round(imagePoints[i].x);
            auto row = round(imagePoints[i].y);
            auto x = cloud_cam->points[i].x;
            auto y = cloud_cam->points[i].y;
            auto z = cloud_cam->points[i].z;

            if (z <= 0)
            {
                continue;
            }


            bool hide_flag = false;
            if (col >= 0 && col < image.cols && row >= 0 && row < image.rows)
            {
               if( image.at<cv::Vec3b>(row, col)[2]==0 && image.at<cv::Vec3b>(row, col)[1]==0 && image.at<cv::Vec3b>(row, col)[0]==0)
               {
                   continue;
               }

                if (depth_2.at<float>(row, col) == 0)
                    depth_2.at<float>(row, col) = x * x + y * y + z * z;
                else if (x * x + y * y + z * z < depth_2.at<float>(row, col))
                {
                    depth_2.at<float>(row, col) = x * x + y * y + z * z;
                    hide_flag = true;
                }
            }

            if (col >= 0 && col < image.cols && row >= 0 && row < image.rows && (!range || (x * x + y * y + z * z < range * range)) && !hide_flag)
            {
                pcl::PointXYZRGB point;
                point.x = x;
                point.y = y;
                point.z = z;
                point.r = image.at<cv::Vec3b>(row, col)[2];
                point.g = image.at<cv::Vec3b>(row, col)[1];
                point.b = image.at<cv::Vec3b>(row, col)[0];
                cloud_cam_rgb->points.push_back(point);
            }
        }
        std::cout << cloud_cam_rgb->points.size() <<" can be used "<<"\n";

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_lid_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::transformPointCloud(*cloud_cam_rgb, *cloud_lid_rgb, trans_img_pcd);


        if (cloud_lid_rgb->points.size() != 0)
        {
            cloud_lid_rgb->width = 1;
            cloud_lid_rgb->height = cloud_lid_rgb->points.size();
            // std::cout<<pcd_iter->second;
            std::string name = std::to_string(getNum(pcd_iter->second));
            pcl::io::savePCDFileBinary(output_pcd_path + name + ".pcd", *cloud_lid_rgb);
            std::cout << "Saved " << cloud_lid_rgb->points.size() << " data points to " << name << ".pcd." << std::endl;

           // cv::imwrite(output_img_path + name + ".png", image);
        }
    }

    return 0;
}
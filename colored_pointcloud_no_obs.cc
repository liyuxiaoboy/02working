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
        // std::cout<<tmp<<" ";
        if (tmp < '0' || tmp > '9')
            break;
        num += (tmp - '0') * j;
        j = j * 10;
    }
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
        files[pcd_time] = "/home/moonx/dev/" + name;
        for (int i = 0; i < 7; i++)
        {
            std::string tmp;
            word >> tmp;
            poses[pcd_time][i] = stod(tmp);
        }
        // std::cout << files[pcd_time];
        // printf(" %ld %lf %lf %lf %lf %lf %lf %lf\n", pcd_time, poses[pcd_time][0], poses[pcd_time][1], poses[pcd_time][2], poses[pcd_time][3], poses[pcd_time][4], poses[pcd_time][5], poses[pcd_time][6]);
    }
    fin.clear();
    fin.close();
}

void getImgFilesAndPOses(const std::string &image_path, std::map<long, std::string> &files, const std::string &poses_path, std::map<long, double[7]> &poses)
{
    std::ifstream fin(poses_path, std::ios::in);
    char line[1024] = {0};
    std::string name = "";
    std::string timestamp = "";
    while (fin.getline(line, sizeof(line)))
    {
        std::stringstream word(line);
        word >> name;
        word >> timestamp;
        long img_time = stod(timestamp) * 1e6;
        files[img_time] = image_path + timestamp + ".png";
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

    // pcl::PassThrough<pcl::PointXYZ> pass;
    // pass.setInputCloud(cloud_f);
    // pass.setFilterFieldName("z");
    // pass.setFilterLimits(0.0, 1000);
    // //pass.setFilterLimitsNegative (true);
    // std::cout<<cloud->points.size()<<" " <<20000<< " "<<cloud->points[20000].z<<"\n";
    // pass.filter(*cloud);
    // std::cout<<cloud->points.size()<<" "<<20000<< " "<<cloud->points[20000].z<<"\n";
    for (int i = 0; i < cloud->points.size(); i++)
    {
        points.push_back(cv::Point3d(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z));
    }

    return points;
}

//get tranform from pcd time to image time
void getTransform(long idx_1, long idx_2, Eigen::Affine3d &T, std::map<long, double[7]> &poses_map, std::map<long, double[7]> &img_time_poses_map)
{
    Eigen::Quaterniond q_1(poses_map[idx_1][6], poses_map[idx_1][3], poses_map[idx_1][4], poses_map[idx_1][5]);
    Eigen::Quaterniond q_2(img_time_poses_map[idx_2][6], img_time_poses_map[idx_2][3], img_time_poses_map[idx_2][4], img_time_poses_map[idx_2][5]);

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
    if (argc != 8)
    {
        std::cout << "please input with format: ./color_1 [pcd_poses_path] [img_dir/] [lidarpose_at_img_time.txt] [output_pcd_dir/] [output_image_dir/] [range limitation, 0 means no limitation] [obs_txt_folder]\n";
        return 0;
    }

    std::string pcd_path = argv[1];
    std::string img_path = argv[2];
    std::string lidar_poses_at_img_time = argv[3];
    std::string output_pcd_path = argv[4];
    std::string output_img_path = argv[5];
    double range = std::stod(argv[6]);
    std::string obs_dir = argv[7];

    std::map<long, std::string> pcd_map;
    std::map<long, std::string> img_map;
    std::map<long, double[7]> pcd_poses;
    std::map<long, double[7]> img_time_poses;

    getPcdFilesAndPoses(pcd_path, pcd_map, pcd_poses);
    getImgFilesAndPOses(img_path, img_map, lidar_poses_at_img_time, img_time_poses);

    //transform between lidar and camera
    Eigen::Matrix4d trans_l_c, trans_c_l;
    Eigen::Quaterniond q_c_l(0.497558, -0.433905, 0.495593, -0.564402);
    Eigen::Affine3d extrinsic_l_c;
    extrinsic_l_c.translation() = Eigen::Vector3d(0.378653, -0.14088, -0.567647);
    extrinsic_l_c.linear() = q_c_l.matrix();
    extrinsic_l_c = extrinsic_l_c.inverse();
    trans_l_c = extrinsic_l_c.matrix();
    trans_c_l = extrinsic_l_c.inverse().matrix();

    // std::cout<<trans_l_c<<"\n";

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

    int flag = 0;

    cv::Mat image;

    auto img_iter = img_map.begin();
    for (auto pcd_iter = pcd_map.begin(); pcd_iter != pcd_map.end(); pcd_iter++)
    {
        long img_time;
        if (!getClosestImage(pcd_iter->first, img_map, img_iter, img_time))
            continue;

        cv::Mat img;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_iter->second, *cloud_src);

        //todo transform from pcd's time to img's time
        Eigen::Matrix4d trans_pcd_img;
        Eigen::Matrix4d trans_img_pcd;
        Eigen::Affine3d affine_p_i;
        getTransform(pcd_iter->first, img_time, affine_p_i, pcd_poses, img_time_poses);
        Eigen::Affine3d affine_l_c = extrinsic_l_c * affine_p_i;

        // std::cout << "\n Before: \n" << (extrinsic_l_c * affine_p_i).matrix() << "\n";
        // std::cout << "\n After: \n" << (affine_p_i * extrinsic_l_c).matrix() << "\n";
        trans_pcd_img = affine_l_c.matrix();
        trans_img_pcd = affine_l_c.inverse().matrix();

        // std::cout << pcd_iter->first <<" "<<img_time<<"\n";
        // std::cout << affine_p_i.matrix() << "\n";

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cam(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud_src, *cloud_cam, trans_pcd_img);

        std::vector<cv::Point3d> objectPoints = get3DPoints(cloud_cam);
        // std::cout<<"cloud_src size: "<<cloud_src->points.size()<<"\n";
        cv::projectPoints(objectPoints, rVec, tVec, intrisicMat, distCoeffs, imagePoints);
        // std::cout<<"imagePoints size: "<<imagePoints.size()<<"\n";

        image = cv::imread(img_map[img_time], CV_LOAD_IMAGE_COLOR);
        // std::cout<<"111\n";
        cv::Mat depth_2 = cv::Mat::zeros(image.cols, image.rows, CV_32FC1);
        // std::cout<<"222\n";
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cam_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);

        std::string obs_path = obs_dir + std::to_string(getNum(pcd_iter->second)) + ".txt";
        std::vector<int> obstacle_id;
        obstacle_id = GetObstacleID(obs_path);
        int obs_idx = 0;

        for (int i = 0; i < imagePoints.size(); i++)
        {
            auto col = round(imagePoints[i].x);
            auto row = round(imagePoints[i].y);
            auto x = cloud_cam->points[i].x;
            auto y = cloud_cam->points[i].y;
            auto z = cloud_cam->points[i].z;

            if (i == obstacle_id[obs_idx])
            {
                obs_idx++;
                continue;
            }

            if (z <= 0)
            {
                continue;
            }

            // std::cout<<"col: "<<col<<" row: "<<row<<" x: "<<x<<" y: "<<y<<" z: "<<z<<"\n";

            bool hide_flag = false;
            if (col >= 0 && col < image.cols && row >= 0 && row < image.rows)
            {
                if (depth_2.at<float>(row, col) == 0)
                    depth_2.at<float>(row, col) = x * x + y * y + z * z;
                else if (x * x + y * y + z * z < depth_2.at<float>(row, col))
                {
                    depth_2.at<float>(row, col) = x * x + y * y + z * z;
                    hide_flag = true;
                    // std::cout<<"hide"<<"\n";
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
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_lid_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::transformPointCloud(*cloud_cam_rgb, *cloud_lid_rgb, trans_img_pcd);

        // std::cout<<cloud_cam_rgb->points.size()<<std::endl;
        // std::cout<<pcd_iter->second<<std::endl;
        if (cloud_lid_rgb->points.size() != 0)
        {
            cloud_lid_rgb->width = 1;
            cloud_lid_rgb->height = cloud_lid_rgb->points.size();
            // std::cout<<pcd_iter->second;
            std::string name = std::to_string(getNum(pcd_iter->second));
            pcl::io::savePCDFileBinary(output_pcd_path + name + ".pcd", *cloud_lid_rgb);
            std::cout << "Saved " << cloud_lid_rgb->points.size() << " data points to " << name << ".pcd." << std::endl;

            cv::imwrite(output_img_path + name + ".png", image);
        }
    }

    return 0;
}
/******************************************************************************
 * Copyright 2018 The MoonX Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "gflags/gflags.h"
#include "modules/localization/msf/common/io/velodyne_utility.h"

DEFINE_string(pcd_files_folder, "", "pcd files folder");
DEFINE_string(pcd_poses_path, "", "pcd poses path");
DEFINE_string(output_pcd_path, "", "output pcd path");

void GetPose(const std::string &path, std::map<int, Eigen::Affine3d> *map) {
  std::vector<Eigen::Affine3d> poses_vec;
  std::vector<double> time_stamps;
  std::vector<unsigned int> pcd_indices;
  apollo::localization::msf::velodyne::LoadPcdPoses(path, &poses_vec,
                                                    &time_stamps, &pcd_indices);
  for (unsigned int i = 0; i < time_stamps.size(); i++) {
    (*map)[pcd_indices[i]] = poses_vec[i];
  }
}


int main(int argc, char **argv)
{
    google::ParseCommandLineFlags(&argc, &argv, true);
    std::string pcd_dir = FLAGS_pcd_files_folder;
    std::string odom_path = FLAGS_pcd_poses_path;
    std::string output_path = FLAGS_output_pcd_path;

    std::map<int, Eigen::Affine3d> poses_map;
    GetPose(odom_path, &poses_map);

    long unsigned int a=0;
    for ( auto pcd_iter = poses_map.begin() ; pcd_iter != poses_map.end(); pcd_iter++)
    {   
        if(a<800)
        {
            a++;
            continue;
        }

        if(a>1000)
        {
            break;
        }

        std::string path_tmp = pcd_dir + std::to_string(pcd_iter->first) + ".pcd";
        std::ifstream fin(path_tmp);
        if (!fin) {
        continue;
        }

        if(a>10 && a<poses_map.size()-10)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::io::loadPCDFile(path_tmp, *cloud_base);
            auto begin_iter=pcd_iter;
            auto end_iter=pcd_iter;
            for(int count = 0;count<50;count++)
            {
                 //begin_iter-- ;
                 end_iter++ ;
            }
            end_iter++;
            for(auto j = begin_iter; j != end_iter; j++ )
            {
                if(j==pcd_iter)
                {
                    continue;
                }

                std::string j_tmp = pcd_dir + std::to_string(j->first) + ".pcd";
                std::ifstream fin_j(j_tmp);
                if (!fin_j) {
                continue;
                }
              
                Eigen::Affine3d trans;
                trans = poses_map[pcd_iter->first].inverse() * poses_map[j->first];

                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_j(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::io::loadPCDFile(j_tmp, *cloud_j);
                pcl::transformPointCloud(*cloud_j, *cloud_tmp, trans);
                *cloud_base += *cloud_tmp;
            }
            std::string name = std::to_string(pcd_iter->first);
            pcl::io::savePCDFileBinary(output_path + name + ".pcd", *cloud_base);
        }
        else
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::io::loadPCDFile(path_tmp, *cloud_tmp);
            std::string name = std::to_string(pcd_iter->first);
            pcl::io::savePCDFileBinary(output_path + name + ".pcd", *cloud_tmp);

        }
        a++;
    }

    

    return 0;
}
#include <iostream>  
#include <pcl/filters/voxel_grid.h> 
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>

int main (int argc, char** argv) { 

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filter_tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile("merge4.pcd", *cloud_tmp);

    pcl::VoxelGrid<pcl::PointXYZRGB> sor; 
    sor.setInputCloud (cloud_tmp); sor.setLeafSize (0.02f, 0.02f, 0.02f); 
    sor.filter (*filter_tmp); 
    pcl::io::savePCDFileBinary("table_scene_lms400_downsampled.pcd", *filter_tmp);
     
    return (0); 
    }

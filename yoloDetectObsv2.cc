#include "modules/perception/obstacle/camera/detector/yolo_camera_detector/yolo_camera_detector.h"

#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include <iostream>
#include <string>
#include<fstream>
#include <sstream>

#include "caffe/caffe.hpp"
#include "gtest/gtest.h"
#include "opencv2/opencv.hpp"

#include "modules/common/log.h"
#include "modules/common/util/file.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/cuda_util/region_output.h"
#include "modules/perception/obstacle/camera/detector/common/feature_extractor.h"
#include "modules/perception/obstacle/camera/detector/yolo_camera_detector/proto/yolo.pb.h"
#include "modules/perception/obstacle/camera/detector/yolo_camera_detector/util.h"
#include "modules/perception/obstacle/camera/interface/base_camera_detector.h"

DECLARE_string(yolo_config_filename);

DEFINE_string(test_dir,
              "/apollo/modules/perception/data/yolo_camera_detector_test/",
              "test data directory");

namespace apollo {
namespace perception {

using apollo::common::util::GetProtoFromASCIIFile;
using apollo::common::util::SetProtoToASCIIFile;
using apollo::perception::obstacle::yolo::YoloParam;
class YoloCameraDetectorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    FLAGS_config_manager_path = FLAGS_test_dir + "config_manager.config",
    FLAGS_yolo_config_filename = "config.pt";
    RegisterFactoryYoloCameraDetector();
  }
};
void LoadImages(const std::string &strFile, std::vector<std::string> &vstrImageFilenames)
{
    std::ifstream f;
    f.open(strFile.c_str());
//如果需要跳几行就激活
    // skip first three lines
    //std::string s0;
    //getline(f,s0);
    //getline(f,s0);
    //getline(f,s0);
    while(!f.eof())
    {
        std::string s;
        getline(f,s);
        if(!s.empty())
        {
//注意txt中的空格不读的
            std::stringstream ss;
            ss << s;
            std::string img_name;
            std::string stamp;
            ss >> img_name;
            vstrImageFilenames.push_back(img_name);
            ss >> stamp;
        }
    }
}

TEST_F(YoloCameraDetectorTest, multi_task_test) 
{

//init
  BaseCameraDetector *camera_detector =
      BaseCameraDetectorRegisterer::GetInstanceByName("YoloCameraDetector");
  CHECK(camera_detector->Init());
  CHECK_EQ(camera_detector->Name(), "YoloCameraDetector");

//set image_file
  const std::string image_file = FLAGS_test_dir + "img_timestamps.txt";
  ADEBUG << "test image file: " << image_file;
  std::vector<std::string> vstrImageFilenames; 
  LoadImages(image_file, vstrImageFilenames);
  std::cout<<"############ "<<vstrImageFilenames.size()<<"############"<<std::endl;
  std::cout<<"############ "<<vstrImageFilenames[1]<<"############"<<std::endl;
    for(int j=0;j<vstrImageFilenames.size();j++)
    {

        //img read
        const std::string input_img_path = FLAGS_test_dir + "img/"+vstrImageFilenames[j]+".png";  
        cv::Mat frame = cv::imread(input_img_path, CV_LOAD_IMAGE_COLOR);
        CHECK_NOTNULL(frame.data);
        //detect
        CameraDetectorOptions options;
        CHECK_EQ(camera_detector->Multitask(frame, options, NULL, NULL), false);
        std::vector<std::shared_ptr<VisualObject>> objects;
        cv::Mat lane_map(frame.rows, frame.cols, CV_32FC1);
        CHECK(camera_detector->Multitask(frame, options, &objects, &lane_map));
        ADEBUG << "#objects detected = " << objects.size();

        cv::Mat obs_img=frame.clone();

        //draw picture   
        for(int i=0;i<objects.size();i++)
            {
                Eigen::Vector2f left_uper=objects[i]->upper_left;
                Eigen::Vector2f right_lower=objects[i]->lower_right;
                std::cout << "object points = #######################" << left_uper(0)<<" "<<left_uper(1)<<"############################"<<std::endl;
                std::cout << "object points = #######################" << right_lower(0)<<" "<<right_lower(1)<<"############################"<<std::endl;

                obs_img(cv::Rect(left_uper(0),left_uper(1),right_lower(0)-left_uper(0),right_lower(1)-left_uper(1))).setTo(0);
            }
            std::cout<<"############ obs is ok ############"<<std::endl;

        const std::string obs_map_result_file = FLAGS_test_dir + "opt/"+vstrImageFilenames[j]+".png";  
        cv::imwrite(obs_map_result_file, obs_img);
    }
}
                        }
                            }
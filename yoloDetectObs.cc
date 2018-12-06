#include "modules/perception/obstacle/camera/detector/yolo_camera_detector/yolo_camera_detector.h"

#include <string>
#include <vector>

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

TEST_F(YoloCameraDetectorTest, multi_task_test) {
  BaseCameraDetector *camera_detector =
      BaseCameraDetectorRegisterer::GetInstanceByName("YoloCameraDetector");
  CHECK(camera_detector->Init());
  CHECK_EQ(camera_detector->Name(), "YoloCameraDetector");

  //const std::string image_file = FLAGS_test_dir + "test.jpg";
  const std::string image_file = FLAGS_test_dir + "805.png";
  ADEBUG << "test image file: " << image_file;

  cv::Mat frame = cv::imread(image_file, CV_LOAD_IMAGE_COLOR);
  CHECK_NOTNULL(frame.data);

  CameraDetectorOptions options;
  CHECK_EQ(camera_detector->Multitask(frame, options, NULL, NULL), false);

  std::vector<std::shared_ptr<VisualObject>> objects;
  cv::Mat lane_map(frame.rows, frame.cols, CV_32FC1);
  CHECK(camera_detector->Multitask(frame, options, &objects, &lane_map));
  ADEBUG << "#objects detected = " << objects.size();

  cv::Mat obs_img=frame.clone();
  //obs_img(cv::Rect(0,0,100,100)).setTo(0);
    
  for(int i=0;i<objects.size();i++)
    {
        Eigen::Vector2f left_uper=objects[i]->upper_left;
        Eigen::Vector2f right_lower=objects[i]->lower_right;
        std::cout << "object points = #######################" << left_uper(0)<<" "<<left_uper(1)<<"################################"<<std::endl;
        std::cout << "object points = #######################" << right_lower(0)<<" "<<right_lower(1)<<"################################"<<std::endl;

        obs_img(cv::Rect(left_uper(0),left_uper(1),right_lower(0)-left_uper(0),right_lower(1)-left_uper(1))).setTo(0);
    }
    std::cout<<"############ obs is ok"<<std::endl;
//   const std::string obs_map_result_file = FLAGS_test_dir + "805_obs.png";
   const std::string obs_map_result_file = FLAGS_test_dir + "805_obs.png";
   //cv::imwrite("/apollo/modules/perception/data/yolo_camera_detector_test/805_obs.png", obs_img);
   cv::imwrite(obs_map_result_file, obs_img);


        // Eigen::Vector2f left_uper=objects[0]->upper_left;
        // Eigen::Vector2f right_lower=objects[0]->lower_right;
        // std::cout << "object points = #######################" << left_uper(0)<<" "<<left_uper(1)<<"################################"<<std::endl;
        // std::cout << "object points = #######################" << right_lower(0)<<" "<<right_lower(1)<<"################################"<<std::endl;

        // obs_img(cv::Rect(left_uper(0),left_uper(1),right_lower(0)-left_uper(0),right_lower(1)-left_uper(1))).setTo(0);
    
//   CHECK_EQ(objects.size(), 1);  // Related to current model and threshold

//   const std::string lane_map_result_file = FLAGS_test_dir + "805_detect.png";
//   const std::string obs_map_result_file = FLAGS_test_dir + "805_obs.png";
//   lane_map.convertTo(lane_map, CV_8UC3, 255.0f);
//   cv::imwrite(lane_map_result_file, lane_map);
//   cv::imwrite(obs_map_result_file, obs_img);
}
                        }
                            }
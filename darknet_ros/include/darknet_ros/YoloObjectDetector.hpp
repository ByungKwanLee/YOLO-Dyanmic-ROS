/*
 * YoloObjectDetector.h
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 *
 *  Edited on : 19/01/18
 *      Author : LBK
 */

#pragma once

//LBK
#include<eigen3/Eigen/Dense>

// c++
#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include <pthread.h>
#include <thread>
#include <mutex>
#include <chrono>

// ROS
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int8.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
// #include <lane_msgs/lane_array.h>

// OpenCv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <cv_bridge/cv_bridge.h>

// darknet_ros_msgs
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

// Darknet.
#ifdef GPU
#include "cuda_runtime.h"
#include "curand.h"
#include "cublas_v2.h"
#endif

// extern darknet yolo
extern "C" {
#include "network.h"
#include "detection_layer.h"
#include "region_layer.h"
#include "cost_layer.h"
#include "utils.h"
#include "parser.h"
#include "box.h"
#include "darknet_ros/image_interface.h"
#include <sys/time.h>
}

extern "C" void ipl_into_image(IplImage* src, image im);
extern "C" image ipl_to_image(IplImage* src);
extern "C" void show_image_cv(image p, const char *name, IplImage *disp);

using namespace std;

namespace darknet_ros {

typedef struct
{
  IplImage* image;
  std_msgs::Header header;
} IplImageWithHeader_;

class YoloObjectDetector
{
 public:

  explicit YoloObjectDetector(ros::NodeHandle nh);

  ~YoloObjectDetector();

 private:

  bool readParameters();

  void init();

  void cameraCallback(const sensor_msgs::CompressedImageConstPtr& msg);

  bool publishDetectionImage(const cv::Mat& detectionImage);
  bool publish_compressed_image(const cv::Mat& compressed_image); // LBK EDIT

  //! ROS node handle.
  ros::NodeHandle nodeHandle_;
  ros::Timer timer;

  //! Class labels.
  int numClasses_;
  std::vector<std::string> classLabels_;

  //! Advertise and subscribe to image topics.
  //image_transport::ImageTransport imageTransport_; LBK EDIT

  //! ROS subscriber and publisher.
  //image_transport::Subscriber imageSubscriber_; LBK EDIT
  ros::Subscriber compressed_sub;
  // ros::Subscriber lane_sub;
  ros::Publisher objectPublisher_;
  ros::Publisher boundingBoxesPublisher_;

  //! Camera related parameters.
  int frameWidth_;
  int frameHeight_;

  //! Publisher of the bounding box image.
  ros::Publisher detectionImagePublisher_;
  ros::Publisher detectionCompressedImagePublisher; // LBK EDIT

  // Yolo running on thread.
  std::thread yoloThread_;

  // Darknet.
  char **demoNames_;
  image **demoAlphabet_;
  int demoClasses_;

  network *net_;
  std_msgs::Header headerBuff_[3];
  image buff_[3];
  image buffLetter_[3];

  int buffIndex_ = 0;
  IplImage * ipl_;
  float demoThresh_ = 0;
  float demoHier_ = .5;

  int demoFrame_ = 3;
  float **predictions_;
  int demoIndex_ = 0;
  int demoDone_ = 0;
  float *avg_;
  int demoTotal_ = 0;

  std_msgs::Header imageHeader_;
  cv::Mat camImageCopy_;

  bool imageStatus_ = false;
  boost::shared_mutex mutexImageStatus_;

  bool isNodeRunning_ = true;
  boost::shared_mutex mutexNodeStatus_;

  // LBK boost 
  boost::shared_mutex mutex_buffLetter_;
  boost::shared_mutex mutexImageCallback_;
  // boost::shared_mutex mutexlaneCallback_;
  boost::shared_mutex mutexpublishcompressedCallback_;


  int sizeNetwork(network *net);

  void rememberNetwork(network *net);

  detection *avgPredictions(network *net, int *nboxes);

  void *detectInThread();

  void *fetchInThread();

  void *displayInThread(void *ptr);

  void *displayLoop(void *ptr);

  void *detectLoop(void *ptr);

  void setupNetwork(char *cfgfile, char *weightfile, char *datafile, float thresh,
                    char **names, int classes,
                    int avg_frames, float hier);

  void yolo();

  IplImageWithHeader_ getIplImageWithHeader();

  bool getImageStatus(void);

  bool isNodeRunning(void);

  void publishInThread();


/*
*
* LBK Bong Edit
*
*
*/
  int nboxes = 0;
  int count = 0; // Actual Box number
  int id = 0;
  int keeping_value = 1;
  int image_height;
  int image_width;
  int pre_count = 0;
  int min = 0;
  int detect_memory_number = 0;

  bool lane_activation = false;
  bool one_prediction = false;
  bool activation=false;
  bool callback_available =false;

  detection *dets = 0;
  
  // distance
  float dist[100];

  // keeping tracking count without detection
  int tracking_keeping[100];

  int pre_class[100];
  int now_class[100];

  int pre_object[100];
  int now_object[100];

  // global frame, LBK
  float x_global[100];
  float y_global[100];

  //tracking, LBK
  float pre_xmin[100];
  float pre_xmax[100];
  float pre_ymin[100];
  float pre_ymax[100];

  float now_xmin[100];
  float now_xmax[100];
  float now_ymin[100];
  float now_ymax[100];

  float ** pre = new float*[4];
  float ** now = new float*[4];

  //LBK struct for prediction
  struct nonzero_struct
  {
    int length;
    int id_array[100];
  };
  nonzero_struct tracking_str;
  nonzero_struct nonzero_len_of_(int * array);

  // lane information
  struct lane_struct
  {
    std::string Class;
    std::vector<int> xs, ys;
    std::vector<float> coeff;
    std::vector<float> topview_mat;
    std::vector<float> topview_invmat;
  };
  lane_struct * lane_struct_array = new lane_struct[2];
  lane_struct lane_clone_left, lane_clone_right;

  std::string classification_lane(bool left_in, bool right_in);
  std::string which_lane_is_object_on(int index, float** pre);

  // void laneCallback(const lane_msgs::lane_arrayConstPtr& msg);
  void distance(float * dist, float **now, int size);
  void tracking_thread(const ros::TimerEvent& event);

  int * minimum_index(image im, float ** pre, float ** now, float threshhold);

  float inter_lane_func(float y, lane_struct struct_obj);
  float line_lane_func(float yp, float slope,
              YoloObjectDetector::lane_struct struct_obj, std::string str);
  float slope_mean(
                YoloObjectDetector::lane_struct struct_obj, std::string str);
  
  Eigen::Vector2f topview_conversion(float x, float y, std::vector<float> topview_mat);
  Eigen::MatrixXf ** kalmanfilter(image im, float ** pre, float ** now, int* min_index);
  Eigen::MatrixXf **  prediction(image im, float ** point);
  Eigen::MatrixXf **  alloc_P_pointer;
  Eigen::MatrixXf * alloc_P_min;
  Eigen::MatrixXf * alloc_P_max;

};

} /* namespace darknet_ros*/

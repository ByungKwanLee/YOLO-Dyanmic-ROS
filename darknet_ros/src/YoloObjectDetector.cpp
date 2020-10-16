/*
 * YoloObjectDetector.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 *
 *  Edited on : 19/01/18
 *      Author : LBK
 */

// yolo object detector
#include "darknet_ros/YoloObjectDetector.hpp"

// Check for xServer
#include <X11/Xlib.h>

// C++
#include<iostream>
#include <time.h>
#ifdef DARKNET_FILE_PATH
std::string darknetFilePath_ = DARKNET_FILE_PATH;
#else
#error Path of darknet repository is not defined in CMakeLists.txt.
#endif

namespace darknet_ros {

char *cfg;
char *weights;
char *data;
char **detectionNames;

YoloObjectDetector::YoloObjectDetector(ros::NodeHandle nh)
    : nodeHandle_(nh),
      numClasses_(0),
      classLabels_(0)
{
  ROS_INFO("[YoloObjectDetector] Node started.");

  // Read parameters from config file.
  if (!readParameters()) {
    ros::requestShutdown();
  }

  init();
}

YoloObjectDetector::~YoloObjectDetector()
{
  {
    boost::unique_lock<boost::shared_mutex> lockNodeStatus(mutexNodeStatus_);
    isNodeRunning_ = false;
  }
  yoloThread_.join();
}

bool YoloObjectDetector::readParameters()
{
  // Check if Xserver is running on Linux.
  if (XOpenDisplay(NULL)) {
    // Do nothing!
    ROS_INFO("[YoloObjectDetector] Xserver is running.");
  } else {
    ROS_INFO("[YoloObjectDetector] Xserver is not running.");
  }

  // Set vector sizes.
  nodeHandle_.param("yolo_model/detection_classes/names", classLabels_,
                    std::vector<std::string>(0));
  numClasses_ = classLabels_.size();


  return true;
}

void YoloObjectDetector::init()
{
  ROS_INFO("[YoloObjectDetector] init().");

  // Initialize deep network of darknet.
  std::string weightsPath;
  std::string configPath;
  std::string dataPath;
  std::string configModel;
  std::string weightsModel;

  // Threshold of object detection.
  float thresh;
  nodeHandle_.param("yolo_model/threshold/value", thresh, (float) 0.3);

  // Path to weights file.
  nodeHandle_.param("yolo_model/weight_file/name", weightsModel,
                    std::string("yolov3.weights"));
  nodeHandle_.param("weights_path", weightsPath, std::string("/default"));
  weightsPath += "/" + weightsModel;
  weights = new char[weightsPath.length() + 1];
  strcpy(weights, weightsPath.c_str());

  // Path to config file.
  nodeHandle_.param("yolo_model/config_file/name", configModel, std::string("yolov3.cfg"));
  nodeHandle_.param("config_path", configPath, std::string("/default"));
  configPath += "/" + configModel;
  cfg = new char[configPath.length() + 1];
  strcpy(cfg, configPath.c_str());

  // Path to data folder.
  dataPath = darknetFilePath_;
  dataPath += "/data";
  data = new char[dataPath.length() + 1];
  strcpy(data, dataPath.c_str());

  // Get classes.
  detectionNames = (char**) realloc((void*) detectionNames, (numClasses_ + 1) * sizeof(char*));
  for (int i = 0; i < numClasses_; i++) {
    detectionNames[i] = new char[classLabels_[i].length() + 1];
    strcpy(detectionNames[i], classLabels_[i].c_str());
  }

  // Load network.
  setupNetwork(cfg, weights, data, thresh, detectionNames, numClasses_, 1, 0.5);
  yoloThread_ = std::thread(&YoloObjectDetector::yolo, this);

  // Initialize publisher and subscriber.
  std::string cameraTopicName;
  int cameraQueueSize;
  std::string objectDetectorTopicName;
  int objectDetectorQueueSize;
  bool objectDetectorLatch;
  std::string boundingBoxesTopicName;
  int boundingBoxesQueueSize;
  bool boundingBoxesLatch;
  std::string detectionImageTopicName;
  std::string detectionCompressedImageTopicName; // LBK EDIT
  int detectionImageQueueSize;
  int detectionCompressedImageQueueSize; // LBK EDIT
  bool detectionImageLatch;
  bool detectionCompressedImageLatch; // LBK EDIT

  nodeHandle_.param("subscribers/camera_reading/topic", cameraTopicName,
                    std::string("/camera/image_raw"));
  nodeHandle_.param("subscribers/camera_reading/queue_size", cameraQueueSize, 1);
  nodeHandle_.param("publishers/object_detector/topic", objectDetectorTopicName,
                    std::string("found_object"));
  nodeHandle_.param("publishers/object_detector/queue_size", objectDetectorQueueSize, 1);
  nodeHandle_.param("publishers/object_detector/latch", objectDetectorLatch, false);
  nodeHandle_.param("publishers/bounding_boxes/topic", boundingBoxesTopicName,
                    std::string("bounding_boxes"));
  nodeHandle_.param("publishers/bounding_boxes/queue_size", boundingBoxesQueueSize, 1);
  nodeHandle_.param("publishers/bounding_boxes/latch", boundingBoxesLatch, false);
  nodeHandle_.param("publishers/detection_image/topic", detectionImageTopicName,
                    std::string("detection_image"));
  nodeHandle_.param("publishers/detection_image/queue_size", detectionImageQueueSize, 1);
  nodeHandle_.param("publishers/detection_image/latch", detectionImageLatch, true);
  nodeHandle_.param("publishers/compressed_image/topic", detectionCompressedImageTopicName,
                    std::string("detection_image/compressed")); // LBK, EDIT

  nodeHandle_.param("publishers/compressed_image/queue_size", detectionCompressedImageQueueSize,
                    1); // LBK, EDIT
  nodeHandle_.param("publishers/compressed_image/latch", detectionCompressedImageLatch,
                    true); // LBK, EDIT

  compressed_sub = nodeHandle_.subscribe(cameraTopicName, cameraQueueSize,
                                             &YoloObjectDetector::cameraCallback, this);
  // lane_sub = nodeHandle_.subscribe("/lane_detection/lane_array", 1,
  //                                            &YoloObjectDetector::laneCallback, this);
  
  boundingBoxesPublisher_ = nodeHandle_.advertise<darknet_ros_msgs::BoundingBoxes>(
      boundingBoxesTopicName, boundingBoxesQueueSize, boundingBoxesLatch);
  detectionImagePublisher_ = nodeHandle_.advertise<sensor_msgs::Image>(detectionImageTopicName,
                                                                       detectionImageQueueSize,
                                                                       detectionImageLatch);
  detectionCompressedImagePublisher = nodeHandle_.advertise<sensor_msgs::CompressedImage>(detectionCompressedImageTopicName,
                                                                       detectionCompressedImageQueueSize,
                                                                       detectionCompressedImageLatch); // LBK EDIT
  timer = nodeHandle_.createTimer(ros::Duration(1./30), &YoloObjectDetector::tracking_thread, this);//LBK EDIT


}

void YoloObjectDetector::cameraCallback(const sensor_msgs::CompressedImageConstPtr& msg)
{
  ROS_DEBUG("[YoloObjectDetector] USB image received.");


    cv::Mat image = cv::imdecode(cv::Mat(msg->data), 1);
    image_height=image.size().height;
    image_width=image.size().width;

  if (!image.empty()) {
    {
      boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexImageCallback_);
      camImageCopy_ = image.clone();
    }
    {
      boost::unique_lock<boost::shared_mutex> lockImageStatus(mutexImageStatus_);
      imageStatus_ = true;
    }
    frameWidth_ = image.size().width;
    frameHeight_ = image.size().height;
  }
  return;
}


// LBK EDIT
bool YoloObjectDetector::publishDetectionImage(const cv::Mat& detectionImage)
{
  if (detectionImagePublisher_.getNumSubscribers() < 1)
    return false;
  cv_bridge::CvImage cvImage;
  cvImage.header.stamp = ros::Time::now();
  cvImage.header.frame_id = "detection_image";
  cvImage.encoding = sensor_msgs::image_encodings::BGR8;
  cvImage.image = detectionImage;
  detectionImagePublisher_.publish(*cvImage.toImageMsg());
  ROS_DEBUG("Detection image has been published.");
  return true;
}

// LBK EDIT
bool YoloObjectDetector::publish_compressed_image(const cv::Mat& compressed_image)
{
  std::vector<uint8_t> image_buff;
  sensor_msgs::CompressedImage compressed_class;
  compressed_class.header.stamp = ros::Time::now();
  compressed_class.header.frame_id = "compressed_detection_image";
  compressed_class.format = "jpg";
  cv::imencode(".jpg", compressed_image, image_buff);
  compressed_class.data = image_buff;
  detectionCompressedImagePublisher.publish(compressed_class);
  return true;
}

int YoloObjectDetector::sizeNetwork(network *net)
{
  int i;
  int count = 0;
  for(i = 0; i < net->n; ++i){
    layer l = net->layers[i];
    if(l.type == YOLO || l.type == REGION || l.type == DETECTION){
      count += l.outputs;
    }
  }
  return count;
}

void YoloObjectDetector::rememberNetwork(network *net)
{
  int i;
  int count = 0;
  for(i = 0; i < net->n; ++i){
    layer l = net->layers[i];
    if(l.type == YOLO || l.type == REGION || l.type == DETECTION){
      memcpy(predictions_[demoIndex_] + count, net->layers[i].output, sizeof(float) * l.outputs);
      count += l.outputs;
    }
  }
}

detection *YoloObjectDetector::avgPredictions(network *net, int *nboxes)
{
  int i, j;
  int count = 0;
  fill_cpu(demoTotal_, 0, avg_, 1);
  for(j = 0; j < demoFrame_; ++j){
    axpy_cpu(demoTotal_, 1./demoFrame_, predictions_[j], 1, avg_, 1);
  }
  for(i = 0; i < net->n; ++i){
    layer l = net->layers[i];
    if(l.type == YOLO || l.type == REGION || l.type == DETECTION){
      memcpy(l.output, avg_ + count, sizeof(float) * l.outputs);
      count += l.outputs;
    }
  }
  detection *dets = get_network_boxes(net, buff_[0].w, buff_[0].h, demoThresh_, demoHier_, 0, 1, nboxes);
  return dets;
}


//LBK prediction
Eigen::MatrixXf ** YoloObjectDetector::prediction(image im, float ** point)
{

  // P_pointer, P_min, P_max dynamic allocation
  Eigen::MatrixXf **  P_pointer = new Eigen::MatrixXf * [2];
  Eigen::MatrixXf * P_min = new Eigen::MatrixXf[pre_count];
  Eigen::MatrixXf * P_max = new Eigen::MatrixXf[pre_count];

  float * xmin = *point;
  float * xmax = *(point +1);
  float * ymin = *(point +2);
  float * ymax = *(point +3);

  Eigen::Matrix4f F;
  Eigen::Matrix4f Q;
  Eigen::Matrix4f P_;
  Eigen::Matrix4f P;
  Eigen::Vector4f x_;
  Eigen::Vector4f x;

  // initialization F, Q, P
  float modeling_scalar = pow(2,10);
  float point_scalar = pow(2,5);
  F << 1,0,1,0,
       0,1,0,1,
       0,0,1,0,
       0,0,0,1;
  Q = modeling_scalar * Eigen::MatrixXf::Identity(4,4);
  P = point_scalar * Eigen::MatrixXf::Identity(4,4);

  if (!one_prediction)
  {
    for(int i=0;i<pre_count;i++)
    {
        //Prediction
        x << *(xmin+i) * im.w, *(ymin+i) * im.h, 1, 1;
        x_ = F*x;
        P_ = F*P*F.transpose() + Q;
        P_min[i]  = P_;

        //Data change
        *(xmin+i) = (float)x_(0) / im.w;
        *(ymin+i) = (float)x_(1) / im.h;

        //Prediction
        x << *(xmax+i) * im.w, *(ymax+i) * im.h, 1, 1;
        x_ = F*x;
        P_ = F*P*F.transpose() + Q;
        P_max[i] = P_;

        //Data Change
        *(xmax+i) = (float)x_(0) / im.w;
        *(ymax+i) = (float)x_(1) / im.h;

    }

  }
  else
  {

    for(int i=0;i<pre_count;i++)
    {
        P = alloc_P_min[i];

        //Prediction
        x << *(xmin+i) * im.w, *(ymin+i) * im.h, 1, 1;

        x_ = F*x;

        P_ = F*P*F.transpose() + Q;
        P_min[i]  = P_;

        //Data change
        *(xmin+i) = (float)x_(0) / im.w;
        *(ymin+i) = (float)x_(1) / im.h;

        P = alloc_P_max[i];
        //Prediction
        x << *(xmax+i) * im.w, *(ymax+i) * im.h, 1, 1;
        x_ = F*x;
        P_ = F*P*F.transpose() + Q;
        P_max[i] = P_;

        //Data Change
        *(xmax+i) = (float)x_(0) / im.w;
        *(ymax+i) = (float)x_(1) / im.h;

    }

    //Dynamic Memory Remove
    delete[] alloc_P_pointer[0];
    alloc_P_pointer[0] = NULL;
    delete[] alloc_P_pointer[1];
    alloc_P_pointer[1] = NULL;
    delete[] alloc_P_pointer;
    alloc_P_pointer = NULL;
  }

  P_pointer[0] = P_min;
  P_pointer[1] = P_max;

  return P_pointer;
}

//LBK structure return function
YoloObjectDetector::nonzero_struct YoloObjectDetector::nonzero_len_of_(int * array)
{
  struct nonzero_struct str;
  int nonnzero_len=0;
  int id[100];
  for (int i =0; i<pre_count; i++)
  {
    if(array[i] > 0){
      str.id_array[nonnzero_len] = i;
      nonnzero_len++;
    }
  }

  str.length = nonnzero_len;
  return str;
}

// LBK kalmanfilter
Eigen::MatrixXf ** YoloObjectDetector::kalmanfilter(image im, float ** pre, float ** now, int* min_index)
{
  if(min_index == NULL)
    return 0;

  for( int index = 0; index<pre_count; index++)
  {
    if( min_index[index]<0 )
    {
      if(tracking_keeping[index]<keeping_value) tracking_keeping[index]++;
      tracking_keeping[index] %= keeping_value;
    }else{
      tracking_keeping[index] = 0;
    }
  }

  tracking_str = nonzero_len_of_(tracking_keeping);

  // P_pointer, P_min, P_max dynamic allocation
  Eigen::MatrixXf **  P_pointer = new Eigen::MatrixXf * [2];
  Eigen::MatrixXf * P_min = new Eigen::MatrixXf[count + tracking_str.length];
  Eigen::MatrixXf * P_max = new Eigen::MatrixXf[count + tracking_str.length];

  float * xmin = *pre;
  float * xmax = *(pre +1);
  float * ymin = *(pre +2);
  float * ymax = *(pre +3);

  float * nowxmin = *now;
  float * nowxmax = *(now+1);
  float * nowymin = *(now+2);
  float * nowymax = *(now+3);
  float measurement_scalar = pow(2,10);

  Eigen::MatrixXf H(2,4);
  Eigen::MatrixXf R(2,2);
  Eigen::MatrixXf K(4,2);
  Eigen::VectorXf x_(4);
  Eigen::VectorXf x(4);
  Eigen::VectorXf z(2);

  // initialization F, Q, P_min, P_max
  float modeling_scalar = pow(2,5);
  float point_scalar = pow(2,5);

  R = measurement_scalar * Eigen::MatrixXf::Identity(2,2);
  H << 1,0,0,0,
        0,1,0,0;

  for(int i=0; i<count; i++){
    P_min[i] = point_scalar * Eigen::MatrixXf::Identity(4,4);
    P_max[i] = point_scalar * Eigen::MatrixXf::Identity(4,4);
  }
  // std::cout<< "Check "<<pre_count + tracking_str.length <<" < 100 ??"<<std::endl;
  for(int add = 0; add<tracking_str.length; add++){
    // std::cout<< pre_count<<", "<<count<<", "<< add <<", "<< tracking_str.length <<", "<< tracking_str.id_array[add]<<std::endl;
    nowxmin[count+add] = xmin[tracking_str.id_array[add]];
    nowxmax[count+add] = xmax[tracking_str.id_array[add]];
    nowymin[count+add] = ymin[tracking_str.id_array[add]];
    nowymax[count+add] = ymax[tracking_str.id_array[add]];
    P_min[count+add] = alloc_P_min[tracking_str.id_array[add]];
    P_max[count+add] = alloc_P_max[tracking_str.id_array[add]];
    now_object[count+add] = pre_object[tracking_str.id_array[add]];
    now_class[count+add] = pre_class[tracking_str.id_array[add]];
  }
// std::cout<< "kalmanfilter 1"<<std::endl;
  for( int index = 0; index<pre_count; index++)
  {
    if(min_index[index]>=0)
    {

      // min id correctionstd::cout<< "kalmanfilter 1"<<std::endl;
      now_object[min_index[index]] = pre_object[index];

      // min Correction
      x_ << *(xmin+index) * im.w, *(ymin+index) * im.h, 1, 1;
      z << *(nowxmin+min_index[index])*im.w, *(nowymin+min_index[index])* im.h;

      K = alloc_P_min[index]*H.transpose() * (H*alloc_P_min[index]*H.transpose() + R).inverse();

      x = x_ + K*(z-H*x_);
      P_min[min_index[index]] = (Eigen::MatrixXf::Identity(4,4) - K*H)*alloc_P_min[index];

      *(nowxmin+min_index[index]) = x(0) / im.w;
      *(nowymin+min_index[index]) = x(1) / im.h;

      // max Correction
      x_ << *(xmax+index) *im.w, *(ymax+index) *im.h, 1, 1;
      z << *(nowxmax+min_index[index]) *im.w, *(nowymax+min_index[index]) *im.h;

      K = alloc_P_max[index]*H.transpose() * (H*alloc_P_max[index]*H.transpose() + R).inverse();
      x = x_ + K*(z-H*x_);
      P_max[min_index[index]] = (Eigen::MatrixXf::Identity(4,4) - K*H)*alloc_P_max[index];

      *(nowxmax+min_index[index]) = x(0) / im.w;
      *(nowymax+min_index[index]) = x(1) / im.h;
    }
  }

  //Dynamic Allocation Remove
  delete[] alloc_P_pointer[0];
  alloc_P_pointer[0] = NULL;
  delete[] alloc_P_pointer[1];
  alloc_P_pointer[1] = NULL;
  delete[] alloc_P_pointer;
  alloc_P_pointer = NULL;

  P_pointer[0] = P_min;
  P_pointer[1] = P_max;

  return P_pointer;
}


// LBK tracking thread 30hz
void YoloObjectDetector::tracking_thread(const ros::TimerEvent& event)
{

  if(!callback_available)
    return;

  activation = false;
  std::mutex lock_oper;
  lock_oper.lock();

  image display = buff_[(buffIndex_+2) % 3];

  alloc_P_pointer = prediction(display, pre);
  alloc_P_min = alloc_P_pointer[0]; // min
  alloc_P_max = alloc_P_pointer[1]; // max
  distance(dist, pre, pre_count);
  tracking_detections(display, dets, dist, pre, pre_object, pre_class, nboxes, pre_count, demoThresh_, demoNames_, demoAlphabet_, demoClasses_);

  // Publish box and image.
  publishInThread();
  cv::Mat cvImage = cv::cvarrToMat(ipl_);
  // publishDetectionImage(cv::Mat(cvImage));
  publish_compressed_image(cv::Mat(cvImage));

  one_prediction = true;
  activation = true;
  lock_oper.unlock();
  return;
}

// Data Association with box matching
int * YoloObjectDetector::minimum_index(image im, float ** pre, float ** now, float threshhold)
{

  float * pre_xmin = *pre;
  float * pre_xmax = *(pre +1);
  float * pre_ymin = *(pre +2);
  float * pre_ymax = *(pre +3);
  float * now_xmin = *now;
  float * now_xmax = *(now+1);
  float * now_ymin = *(now+2);
  float * now_ymax = *(now+3);

  if (count != 0){
    int * min_index = new int[pre_count]; // Dynamic allocation
    float cost[count];
    float min_value; // LBK EDIT
    int cut_off; //LBK EDIT

    for(int i = 0; i < pre_count; i++){
        for(int j = 0; j < count; j++){
          if (pre_class[i] == now_class[j]){
            cost[j] = sqrt(pow(pre_xmin[i]-now_xmin[j], 2)+
                      pow(pre_xmax[i]-now_xmax[j], 2)+
                      pow(pre_ymin[i]-now_ymin[j], 2)+
                      pow(pre_ymax[i]-now_ymax[j], 2)); // BONG EDIT

          }else{
            cost[j] = -1;
          }
        }
        // printf("\ncost : \n");
        // for (int j=0; j<count;j++){
        //   std::cout<<cost[j]<<" ";
        // }
        // printf("\n");

        for(int j=0;j<count;j++){
          if(cost[j]>=0 && cost[j] < threshhold){
            min_value=cost[j];
            min_index[i] = j;
            cut_off = j;
            break;
          }else{
            min_value=-1;
          }
        }
        if(min_value == -1){ // LBK EDIT, All of cost is negative
          min_index[i] = -1;
        }
        else{

          for(int j=cut_off;j<count;j++){
            if(cost[j]<min_value && cost[j]>=0 && cost[j] < threshhold){
              min_value = cost[j];
              min_index[i] = j;
            }
          }
        }
    }

      // printf("\nmin_index : \n");
      // for (int j=0; j<pre_count;j++){
      //   // std::cout<<min_value<<" ";
      //   std::cout<<min_index[j]<<" ";
      // }
      // printf("\n");

    return min_index;
  }

  return NULL;
}

// LBK EDIT
void *YoloObjectDetector::detectInThread()
{
  // LBK Synchronization for trackint thread
  callback_available = false; // LBK EDIT
  
  float nms = .4;
  layer l = net_->layers[net_->n - 1];

  // LBK detection boost for not sharing variable 
  {
    boost::unique_lock<boost::shared_mutex> lock(mutex_buffLetter_);
    float *X = buffLetter_[(buffIndex_ + 2) % 3].data;
    float *prediction = network_predict(net_, X);
  }
  rememberNetwork(net_);

  // free of dynmaic alloction, which is important operation
  // std::cout<<"activation on"<<std::endl;
  if(activation)
  {   
      free_detections(dets, nboxes);
      detect_memory_number = 0;
      // std::cout<<"remove dets activation off"<<std::endl;
  }else if( detect_memory_number >= 1){
    free_detections(dets, nboxes);
    detect_memory_number = 0;
    // std::cout<<"remove dets activation off"<<std::endl;
  }
  // Key Point
  dets = avgPredictions(net_, &nboxes);
  detect_memory_number ++;

  if (nms > 0) do_nms_obj(dets, nboxes, l.classes, nms);

  // extract the bounding boxes and send them to ROS
  int i, j;
  count = 0;
  if(id > 1000) id = 0 ;
  for (i = 0; i < nboxes; ++i) {
    float xmin = dets[i].bbox.x - dets[i].bbox.w / 2.;
    float xmax = dets[i].bbox.x + dets[i].bbox.w / 2.;
    float ymin = dets[i].bbox.y - dets[i].bbox.h / 2.;
    float ymax = dets[i].bbox.y + dets[i].bbox.h / 2.;

    if(ymax>950)
      continue;

    if (xmin < 0)
      xmin = 0;
    if (ymin < 0)
      ymin = 0;
    if (xmax > 1)
      xmax = 1;
    if (ymax > 1)
      ymax = 1;

    // iterate through possible boxes and collect the bounding boxes
    for (j = 0; j < demoClasses_; ++j) {
      if (j!=2 && j!=5 && j!=7)
        continue;
      if (dets[i].prob[j]>0.6) {
        float x_center = (xmin + xmax) / 2;
        float y_center = (ymin + ymax) / 2;
        float BoundingBox_width = xmax - xmin;
        float BoundingBox_height = ymax - ymin;

        // Car, Truch, Bus -> Car
        if(j==2 || j==5 || j==7){
          now_class[count] = 2;
        }else{
          now_class[count] = j;
        }

        now_xmin[count] = x_center - BoundingBox_width / 2;
        now_xmax[count] = x_center + BoundingBox_width / 2;
        now_ymin[count] = y_center - BoundingBox_height / 2;
        now_ymax[count] = y_center + BoundingBox_height / 2;

        // LBK EDIT
        now_object[count] = id;
        id++; // object id
        // LBK End

        count++;
        break;
      }
    }
  }

  image display = buff_[(buffIndex_+2) % 3];
  if( count!=0 && one_prediction)
  {

    // std::cout<<"pre :"<<std::endl;
    // for(int i=0; i<pre_count;i++){
    //   std::cout<<pre_xmin[i]<<", "<<pre_xmax[i]<<", "<<pre_ymin[i]<<", "<<pre_ymax[i]<<std::endl;
    // }

    // std::cout<<"now :"<<std::endl;
    // for(int i=0; i<count;i++){
    //   std::cout<<now_xmin[i]<<", "<<now_xmax[i]<<", "<<now_ymin[i]<<", "<<now_ymax[i]<<std::endl;
    // }

    //prediction -> matching -> tracking
    alloc_P_pointer = YoloObjectDetector::prediction(display, pre);
    alloc_P_min = alloc_P_pointer[0]; // min
    alloc_P_max = alloc_P_pointer[1]; // max

    int * min_index = minimum_index(display, pre, now, 0.2);

    alloc_P_pointer = kalmanfilter(display, pre, now, min_index);
    alloc_P_min = alloc_P_pointer[0]; // post min
    alloc_P_max = alloc_P_pointer[1]; // post max

    // Dynamic memory data remove
    delete[] min_index;
    min_index = NULL;
  }

  // update
  pre_count = count + tracking_str.length; // LBK BONG EDIT
  // getting distance value
  distance(dist, now, pre_count);
  detections(display, dets, dist, now, now_object, now_class, nboxes, pre_count, demoThresh_, demoNames_, demoAlphabet_, demoClasses_);

  for (int i=0;i<pre_count;i++)
  {
    pre_xmin[i] = now_xmin[i]; // LBK EDIT
    pre_xmax[i] = now_xmax[i]; // LBK EDIT
    pre_ymin[i] = now_ymin[i]; // LBK EDIT
    pre_ymax[i] = now_ymax[i]; // LBK EDIT
    pre_class[i] = now_class[i]; // LBK EDIT
    pre_object[i] = now_object[i]; // LBK EDIT
  }

  // index : prediction (detection)
  demoIndex_ = (demoIndex_ + 1) % demoFrame_;
  callback_available = true; // LBK EDIT

  publishInThread();
  // Publish image.
  cv::Mat cvImage = cv::cvarrToMat(ipl_);
  // publishDetectionImage(cv::Mat(cvImage));
  publish_compressed_image(cv::Mat(cvImage));

  return 0;
}

void *YoloObjectDetector::fetchInThread()
{

  // LBK boost unique_lock for not sharing resource 
  {
    boost::unique_lock<boost::shared_mutex> lock(mutexImageCallback_);
    IplImageWithHeader_ imageAndHeader = getIplImageWithHeader();
    IplImage* ROS_img = cvCloneImage(imageAndHeader.image);
    ipl_into_image(ROS_img, buff_[buffIndex_]);
    cvReleaseImage(&ROS_img);
    headerBuff_[buffIndex_] = imageAndHeader.header;
  }
  rgbgr_image(buff_[buffIndex_]);

  // LBK boost unique_lock for not sharing resource 
  {
    boost::unique_lock<boost::shared_mutex> lock(mutex_buffLetter_);
    letterbox_image_into(buff_[buffIndex_], net_->w, net_->h, buffLetter_[buffIndex_]);
  }

  // LBK shorten code from displaythread to this 
  show_image_cv(buff_[(buffIndex_ + 1)%3], "", ipl_);
  return 0;
}

void YoloObjectDetector::distance(float * dist, float **now, int size)
{

  float * now_xmin = *now;
  float * now_xmax = *(now+1);
  float * now_ymin = *(now+2);
  float * now_ymax = *(now+3);

  Eigen::Vector3f A;
  Eigen::Matrix3f proj;
  Eigen::Vector3f out;
  proj << -5.97433409e-02, -1.56897173e+00,  9.86153126e+02,
       -4.00669186e-02, -1.96269046e+00,  1.22784732e+03,
       -3.55481655e-05, -1.63223988e-03,  1.00000000e+00;

  for(int i=0;i<size;i++)
  {
    A << ( *(now_xmin+i) + *(now_xmax+i) )/2 * image_width, *(now_ymax+i) *image_height, 1;
    out = proj * A;
    out(0) = out(0) / out(2);
    out(1) = out(1) / out(2);
    if(out(0) > 0 && out(0) < image_width && out(1) > 0 && out(1) <image_height){
      *(x_global+i) = (float) 0.08125 * (out(0)-965);
      *(y_global+i) = (float) -0.08125 * (out(1)-1160);
      *(dist+i) = (float) 0.08125 * sqrt(pow(out(0) - 965, 2)+pow( out(1) -1160, 2));
    }else{
      // we don't know
      *(dist+i) = -1001;
      *(x_global+i) = -1001;
      *(y_global+i) = -1001;
    }
  }
}

void YoloObjectDetector::setupNetwork(char *cfgfile, char *weightfile, char *datafile, float thresh,
                                      char **names, int classes,
                                      int avg_frames, float hier)
{
  demoFrame_ = avg_frames;
  image **alphabet = load_alphabet_with_file(datafile);
  demoNames_ = names;
  demoAlphabet_ = alphabet;
  demoClasses_ = classes;
  demoThresh_ = thresh;
  demoHier_ = hier;
  printf("YOLO V3\n");
  net_ = load_network(cfgfile, weightfile, 0);
  set_batch_network(net_, 1);
}

void YoloObjectDetector::yolo()
{
  const auto wait_duration = std::chrono::milliseconds(2000);
  while (!getImageStatus()) {
    printf("Waiting for image.\n");
    if (!isNodeRunning()) {
      return;
    }
    std::this_thread::sleep_for(wait_duration);
  }

  std::thread detect_thread;
  std::thread fetch_thread;

  //structure initialization //LBK
  tracking_str.length = 0;

  // pre dynamic allocation
  *pre = pre_xmin;
  *(pre +1) = pre_xmax;
  *(pre +2) = pre_ymin;
  *(pre +3) = pre_ymax;

  // now dynamic allocation
  *now = now_xmin;
  *(now +1) = now_xmax;
  *(now +2) = now_ymin;
  *(now +3) = now_ymax;

  // original
  srand(2222222);

  int i;
  demoTotal_ = sizeNetwork(net_);
  predictions_ = (float **) calloc(demoFrame_, sizeof(float*));
  for (i = 0; i < demoFrame_; ++i){
      predictions_[i] = (float *) calloc(demoTotal_, sizeof(float));
  }
  avg_ = (float *) calloc(demoTotal_, sizeof(float));
  layer l = net_->layers[net_->n - 1];


  IplImageWithHeader_ imageAndHeader = getIplImageWithHeader();
  IplImage* ROS_img = imageAndHeader.image;
  buff_[0] = ipl_to_image(ROS_img);
  buff_[1] = copy_image(buff_[0]);
  buff_[2] = copy_image(buff_[0]);
  headerBuff_[0] = imageAndHeader.header;
  headerBuff_[1] = headerBuff_[0];
  headerBuff_[2] = headerBuff_[0];
  buffLetter_[0] = letterbox_image(buff_[0], net_->w, net_->h);
  buffLetter_[1] = letterbox_image(buff_[0], net_->w, net_->h);
  buffLetter_[2] = letterbox_image(buff_[0], net_->w, net_->h);
  ipl_ = cvCreateImage(cvSize(buff_[0].w, buff_[0].h), IPL_DEPTH_8U, buff_[0].c);

  while (!demoDone_) {
    buffIndex_ = (buffIndex_ + 1) % 3;
    fetch_thread = std::thread(&YoloObjectDetector::fetchInThread, this);
    detect_thread = std::thread(&YoloObjectDetector::detectInThread, this);
    fetch_thread.join();
    detect_thread.join();
    // ++count;
    if (!isNodeRunning()) {
      demoDone_ = true;
    }
  }

}

IplImageWithHeader_ YoloObjectDetector::getIplImageWithHeader()
{
  IplImage* ROS_img = new IplImage(camImageCopy_);
  IplImageWithHeader_ header = {.image = ROS_img, .header = imageHeader_};
  return header;
}

bool YoloObjectDetector::getImageStatus(void)
{
  boost::shared_lock<boost::shared_mutex> lock(mutexImageStatus_);
  return imageStatus_;
}

bool YoloObjectDetector::isNodeRunning(void)
{
  boost::shared_lock<boost::shared_mutex> lock(mutexNodeStatus_);
  return isNodeRunning_;
}

void YoloObjectDetector::publishInThread()
{
  int num = pre_count;
  darknet_ros_msgs::BoundingBoxes bounding_box_array;
  bounding_box_array.header.stamp = ros::Time::now();
  bounding_box_array.header.frame_id = "yolo_detection_tracking";
  for(int i=0;i<num;i++){
    darknet_ros_msgs::BoundingBox boundingBox;
    boundingBox.Class = classLabels_[pre_class[i]];
    boundingBox.lane_class = "No Lane topic";
    if(lane_activation)
      // boundingBox.lane_class = which_lane_is_object_on(i, pre);

    boundingBox.id = pre_object[i];
    boundingBox.xmin = pre_xmin[i]*image_width;
    boundingBox.xmax = pre_xmax[i]*image_width;
    boundingBox.ymin = pre_ymin[i]*image_height;
    boundingBox.ymax = pre_ymax[i]*image_height;

    boundingBox.distance = dist[i];
    boundingBox.x_global = x_global[i];
    boundingBox.y_global = y_global[i];
    boundingBox.velocity = boundingBox.float_UNKNOWN;

    bounding_box_array.bounding_boxes.push_back(boundingBox);
  }
  boundingBoxesPublisher_.publish(bounding_box_array);

}
// std::string YoloObjectDetector::classification_lane(bool left_in, bool right_in){

//   if (left_in && right_in){
//         return "ego_lane";
//   }
//   else if(left_in){
//     return "right_lane";
//   }
//   else if(right_in){
//     return "left_lane";
//   }
//   return "retry";
// }


// float YoloObjectDetector::inter_lane_func(float y,
//               YoloObjectDetector::lane_struct struct_obj){

//   float x = struct_obj.coeff[0] * pow(y,3) +struct_obj.coeff[1] * pow(y,2)
//     + struct_obj.coeff[2] * y + struct_obj.coeff[3];

//   return x;

// }

// float YoloObjectDetector::line_lane_func(float yp, float slope,
//               YoloObjectDetector::lane_struct struct_obj, std::string str){
//   if (str=="upper"){
//     int size = struct_obj.ys.size();
//     float ys = struct_obj.ys[size-1];
//     float xs = struct_obj.xs[size-1];
//     return slope*(yp-ys) + xs;
//   }
//   else if(str=="lower"){
//     float ys = struct_obj.ys[0];
//     float xs = struct_obj.xs[0];
//     return slope*(yp-ys) + xs;
//   }
//   if(str=="upper_topview"){
//     int size = struct_obj.ys.size();
//     float ys = struct_obj.ys[size-1];
//     float xs = struct_obj.xs[size-1];
//     Eigen::Vector2f po = topview_conversion(xs, ys, struct_obj.topview_mat);
//     return slope*(yp-po(1)) + po(0);
//   }
//   else if(str=="lower_topview"){
//     float ys = struct_obj.ys[0];
//     float xs = struct_obj.xs[0];
//     Eigen::Vector2f po = topview_conversion(xs, ys, struct_obj.topview_mat);
//     return slope*(yp-po(1)) + po(0);
//   }
//   return -1001;
// }

// Eigen::Vector2f YoloObjectDetector::topview_conversion(float x, float y,
//                 std::vector<float> topview_mat)
// {

//   Eigen::Vector3f A;
//   Eigen::Matrix3f proj = Eigen::Map<Eigen::Matrix3f>(topview_mat.data());
//   Eigen::Vector3f out;
//   A << x, y, 1;
//   out = proj * A;
//   out(0) = out(0) / out(2);
//   out(1) = out(1) / out(2);

//   Eigen::Vector2f output;
//   output << out(0), out(1);

//   return output;
// }

// float YoloObjectDetector::slope_mean(
//                 YoloObjectDetector::lane_struct struct_obj, std::string str)
// {

//   if (str=="upper"){

//       std::vector<float> slope;
//       int si = struct_obj.ys.size();
//       int number = 10;
//       if(si<number)
//         number =si - 2;

//       for (int i=0; i<number; i++)
//       {
//         slope.push_back( (float) (struct_obj.xs[si-i-1] - struct_obj.xs[si-i])
//           / (struct_obj.ys[si-i-1] - struct_obj.ys[si-i]));
//       }

//       float slope_ = std::accumulate(slope.begin(), slope.end(), 0.0);
//       slope_ /= number ;
//       slope.clear();

//       return slope_;
//     }
//     else if(str=="lower"){

//     std::vector<float> slope;
//     int si = struct_obj.ys.size();
//     int number = 10;
//     if(si<number)
//       number =si - 2;


//     for (int i=0; i<number; i++)
//     {
//       slope.push_back( (float) (struct_obj.xs[i+1] - struct_obj.xs[i])
//         / (struct_obj.ys[i+1] - struct_obj.ys[i]));
//     }

//     float slope_ = std::accumulate(slope.begin(), slope.end(), 0.0);
//     slope_ /= number ;
//     slope.clear();

//     return slope_;
//   }



//   if (str=="upper_topview"){

//     std::vector<float> slope;
//     int si = struct_obj.ys.size();
//     int number = 10;
//     if(si<10)
//       number =si - 2;

//     for (int i=0; i<number; i++)
//     {
//       Eigen::Vector2f tp_0 = topview_conversion(struct_obj.xs[si-i], struct_obj.ys[si-i],
//               struct_obj.topview_mat);
//       Eigen::Vector2f tp_1 = topview_conversion(struct_obj.xs[si-i-1], struct_obj.ys[si-i-1],
//               struct_obj.topview_mat);

//       slope.push_back( (float) (tp_1(0) - tp_0(0))
//         / (tp_1(1) - tp_0(1)));
//     }

//     float slope_ = std::accumulate(slope.begin(), slope.end(), 0.0);
//     slope_ /= number ;
//     slope.clear();

//     return slope_;
//   }
//   else if(str=="lower_topview"){

//     std::vector<float> slope;
//     int si = struct_obj.ys.size();
//     int number = 10;
//     if(si<10)
//       number =si - 2;


//     for (int i=0; i<number; i++)
//     {
//       Eigen::Vector2f tp_0 = topview_conversion(struct_obj.xs[i], struct_obj.ys[i],
//               struct_obj.topview_mat);
//       Eigen::Vector2f tp_1 = topview_conversion(struct_obj.xs[i+1], struct_obj.ys[i+1],
//               struct_obj.topview_mat);

//       slope.push_back( (float) (tp_1(0) - tp_0(0))
//         / (tp_1(1) - tp_0(1)));
//     }

//     float slope_ = std::accumulate(slope.begin(), slope.end(), 0.0);
//     slope_ /= number ;
//     slope.clear();

//     return slope_;
//   }
//   else{
//     return -1001;
//   }

// }

// std::string YoloObjectDetector::which_lane_is_object_on(int index){
//   float * xmin = *pre;
//   float * xmax = *(pre +1);
//   float * ymin = *(pre +2);
//   float * ymax = *(pre +3);

//   float bmx = (*(xmin+index) + *(xmax+index) ) / 2.;
//   float bmy = *(ymax+index);

//   float resized_bmx = bmx * 640;
//   float resized_bmy = bmy * 400;

//   Eigen::Vector2f cen_top = topview_conversion(resized_bmx, resized_bmy, lane_clone_left.topview_mat);
//   float topview_cx = cen_top(0);
//   float topview_cy = cen_top(1);

//   if (lane_clone_left.Class == "left_lane" && lane_clone_right.Class == "right_lane")
//   {

//     int minimum = lane_clone_left.ys[0];
//     int maximum = lane_clone_left.ys[lane_clone_left.ys.size()-1];

//     // with interval of lane information
//     if ( minimum <= resized_bmy &&  resized_bmy <= maximum){
//       bool left_in = resized_bmx >= inter_lane_func(resized_bmy, lane_clone_left);
//       bool right_in = resized_bmx <= inter_lane_func(resized_bmy, lane_clone_right);

//       std::string final_class = classification_lane(left_in, right_in);
//       if(final_class == "retry"){
//         return "retry, midpoint";
//       }
//       else{
//         return final_class;
//       }
//     }
//     else if(maximum < resized_bmy){

//       float slope_left = slope_mean(lane_clone_left, "upper");
//       float slope_right = slope_mean(lane_clone_right, "upper");

//       bool left_in = resized_bmx >= line_lane_func(resized_bmy,
//                       slope_left, lane_clone_left,"upper");
//       bool right_in = resized_bmx <= line_lane_func(resized_bmy,
//                       slope_right, lane_clone_right,"upper");

//       std::string final_class = classification_lane(left_in, right_in);
//       if(final_class == "retry"){
//         return "retry, maximum";
//       }
//       else{
//         return final_class;
//       }
//     }
//     else{

//       // float slope_left = slope_mean(lane_clone_left, "lower_topview");
//       // float slope_right = slope_mean(lane_clone_right, "lower_topview");

//       // bool left_in = topview_cx >= line_lane_func(topview_cy,
//       //                 slope_left,lane_clone_left,"lower_topview");
//       // bool right_in = topview_cx <= line_lane_func(topview_cy,
//       //                 slope_right,lane_clone_right,"lower_topview");
//       float slope_left = slope_mean(lane_clone_left, "lower");
//       float slope_right = slope_mean(lane_clone_right, "lower");
//       bool left_in = resized_bmx >= line_lane_func(resized_bmy,
//                     slope_left, lane_clone_left,"lower");
//       bool right_in = resized_bmx <= line_lane_func(resized_bmy,
//                     slope_right, lane_clone_right,"lower");
//       std::string final_class = classification_lane(left_in, right_in);


//       if(final_class=="retry"){

//         slope_left = slope_mean(lane_clone_left, "lower_topview");
//         slope_right = slope_mean(lane_clone_right, "lower_topview");

//         left_in = topview_cx >= line_lane_func(topview_cy,
//                         slope_left,lane_clone_left,"lower_topview");
//         right_in = topview_cx <= line_lane_func(topview_cy,
//                         slope_right,lane_clone_right,"lower_topview");
//         final_class = classification_lane(left_in, right_in);
//         // slope_left = slope_mean(lane_clone_left, "lower");
//         // slope_right = slope_mean(lane_clone_right, "lower");
//         // left_in = resized_bmx >= line_lane_func(resized_bmy,
//         //               slope_left, lane_clone_left,"lower");
//         // right_in = resized_bmx <= line_lane_func(resized_bmy,
//         //               slope_right, lane_clone_right,"lower");
//         // final_class = classification_lane(left_in, right_in);
//         return final_class;
//       }
//       else{
//         return final_class;
//       }
//     }
//   }
//   else{
//     return "No complete lane";
//   }

// }


// void YoloObjectDetector::laneCallback(const lane_msgs::lane_arrayConstPtr& msg){

//   if(lane_activation){
//     for(int i = 0 ; i <2; i++){
//       lane_struct_array[i].xs.clear();
//       lane_struct_array[i].ys.clear();
//       lane_struct_array[i].coeff.clear();
//       lane_struct_array[i].topview_mat.clear();
//       lane_struct_array[i].topview_invmat.clear();
//     }
//   }

//   for(int i = 0; i < msg->lane_array.size(); i++){
//     lane_struct_array[i].Class = msg->lane_array[i].Class;
//     lane_struct_array[i].xs = msg->lane_array[i].xs;
//     lane_struct_array[i].ys = msg->lane_array[i].ys;
//     lane_struct_array[i].coeff = msg->lane_array[i].coeff;
//     lane_struct_array[i].topview_mat.assign(msg->trans_mat.begin(),msg->trans_mat.end());
//     lane_struct_array[i].topview_invmat.assign(msg->inv_mat.begin(),msg->inv_mat.end());
//   }


//   {
//     boost::unique_lock<boost::shared_mutex> locklaneCallback(mutexlaneCallback_);
//     lane_clone_left = lane_struct_array[0];
//     lane_clone_right = lane_struct_array[1];
//     lane_activation = true;
//   }
// }


} /* namespace darknet_ros*/
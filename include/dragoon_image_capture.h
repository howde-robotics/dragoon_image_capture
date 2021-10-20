// Author(s): Daniel Bronstein (dbronste@andrew.cmu.edu)
// This file is subject to the terms and conditions defined in the file 'LICENSE',
// which is part of this source code package

#pragma once

#include <stdlib.h>     /* srand, rand */
#include <time.h>   
#include <sys/types.h>
#include <sys/stat.h>

#include <sstream>
#include <stdexcept>

// ROS libraries
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>  
#include <std_msgs/Empty.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// Include other header files
#include <opencv2/opencv.hpp>


class DragoonImageCapture
{
public:

  DragoonImageCapture();

  ~DragoonImageCapture();

private:
  /////////////////////// State variables ///////////////////////
  
  cv::Mat lastDisplayImageMatrix_;
  cv::Mat lastTempImageMatrix_;
  cv::Mat lastRgbImageMatrix_;

  int runID_;
  int frameCount_;

  /////////////////////// calculation variables ///////////////////////

  std::string rgbSaveDirectory_;
  std::string irSaveDirectory_;
  std::string temperatureSaveDirectory_;

  //https://stackoverflow.com/questions/18100097/portable-way-to-check-if-directory-exists-windows-linux-c
  int dirExists(const char* const path);

  // ROS and node related variables
  ros::NodeHandle nh_, private_nh_;

  image_transport::Subscriber displayImageSub_;//uses cv_bridge
  image_transport::Subscriber temperatureImageSub_;//cv bridge
  image_transport::Subscriber rgbImageSub_;//cv bridge

  ros::Subscriber captureCommandSub_;//This should be a service but whatever

  image_transport::Publisher capturedDisplayImagePub_;//uses cv_bridge
  image_transport::Publisher capturedTemperatureImagePub_;//uses cv_bridge
  image_transport::Publisher capturedRGBImagePub_;//uses cv_bridge

  // Boiler plate ROS functions
  void initRos();

  void displayDataCallback(const sensor_msgs::ImageConstPtr& msg);
  void temperatureDataCallback(const sensor_msgs::ImageConstPtr& msg);
  void rgbDataCallback(const sensor_msgs::ImageConstPtr& msg);

  void captureCommandCallback(const std_msgs::Empty& msg);//should be a service
};

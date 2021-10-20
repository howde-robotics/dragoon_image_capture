// Author(s): Daniel Bronstein (dbronste@andrew.cmu.edu)
// This file is subject to the terms and conditions defined in the file 'LICENSE',
// which is part of this source code package

#include "dragoon_image_capture.h"

DragoonImageCapture::DragoonImageCapture() : private_nh_("~")
{
  initRos();
}

void DragoonImageCapture::displayDataCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat displayImage = cv_bridge::toCvShare(msg, "bgra8")->image;
  displayImage.copyTo(lastDisplayImageMatrix_);
}

void DragoonImageCapture::temperatureDataCallback(const sensor_msgs::ImageConstPtr& msg) 
{
  cv::Mat temperatureData = cv_bridge::toCvShare(msg, "32FC1")->image;
  temperatureData.copyTo(lastTempImageMatrix_);
}

void DragoonImageCapture::rgbDataCallback(const sensor_msgs::ImageConstPtr& msg) 
{
  cv::Mat rgbData = cv_bridge::toCvShare(msg, "bgr8")->image;
  rgbData.copyTo(lastRgbImageMatrix_);
}

void DragoonImageCapture::captureCommandCallback(const std_msgs::Empty& msg)
{
  //save last frames if they exist
  std::ostringstream ir_ss;
  ir_ss << irSaveDirectory_ << "IR_run_" << runID_ << "_frame_" << frameCount_ << ".jpg";
  cv::imwrite(ir_ss.str(), lastDisplayImageMatrix_);

  std::ostringstream rgb_ss;
  rgb_ss << rgbSaveDirectory_ << "RGB_run_" << runID_ << "_frame_" << frameCount_ << ".jpg";
  cv::imwrite(rgb_ss.str(), lastRgbImageMatrix_);

  std::ostringstream temp_ss;
  temp_ss << temperatureSaveDirectory_ << "Temperature_run_" << runID_ << "_frame_" << frameCount_ << ".xml";
  cv::FileStorage tempMatFile(temp_ss.str(), cv::FileStorage::WRITE);
  tempMatFile << "matName" << lastTempImageMatrix_;
  tempMatFile.release();

  //publish images to topic for display in rviz
  std_msgs::Header head;
  head.seq = frameCount_;
  head.stamp = ros::Time::now();

  sensor_msgs::ImagePtr displayMsg = cv_bridge::CvImage(head,
    "bgra8", lastDisplayImageMatrix_).toImageMsg();
  capturedDisplayImagePub_.publish(displayMsg);
  
  sensor_msgs::ImagePtr rgbMsg = cv_bridge::CvImage(head,
    "bgr8", lastRgbImageMatrix_).toImageMsg();
  capturedRGBImagePub_.publish(rgbMsg);

  //publish thermography image
  sensor_msgs::ImagePtr tempMsg = cv_bridge::CvImage(head,
      "32FC1", lastTempImageMatrix_).toImageMsg();
  capturedTemperatureImagePub_.publish(tempMsg);
  
  frameCount_++;
}

/******************************************************************************
 * 
 * https://stackoverflow.com/questions/18100097/portable-way-to-check-if-directory-exists-windows-linux-c
 * 
 * Checks to see if a directory exists. Note: This method only checks the
 * existence of the full path AND if path leaf is a dir.
 *
 * @return  >0 if dir exists AND is a dir,
 *           0 if dir does not exist OR exists but not a dir,
 *          <0 if an error occurred (errno is also set)
 *****************************************************************************/
int DragoonImageCapture::dirExists(const char* const path)
{
    struct stat info;

    int statRC = stat( path, &info );
    if( statRC != 0 )
    {
        if (errno == ENOENT)  { return 0; } // something along the path does not exist
        if (errno == ENOTDIR) { return 0; } // something in path prefix is not a dir
        return -1;
    }

    return ( info.st_mode & S_IFDIR ) ? 1 : 0;
}

DragoonImageCapture::~DragoonImageCapture()
{

}

void DragoonImageCapture::initRos()
{

  private_nh_.param<std::string>("rgbSaveDirectory_", rgbSaveDirectory_,"./");
  private_nh_.param<std::string>("irSaveDirectory_", irSaveDirectory_, "./");
  private_nh_.param<std::string>("temperatureSaveDirectory_", temperatureSaveDirectory_, "./");

  //append with directory
  //c++ 11 doesnt have <filesystem> yet
  if (rgbSaveDirectory_.back() != '/')
  {
    rgbSaveDirectory_ += '/';
  }
  if (irSaveDirectory_.back() != '/')
  {
    irSaveDirectory_ += '/';
  }
  if (temperatureSaveDirectory_.back() != '/')
  {
    temperatureSaveDirectory_ += '/';
  }

  //Check for existence
  if (dirExists(rgbSaveDirectory_.c_str()) <= 0)
  {
    ROS_FATAL("Save Directory %s invalid", rgbSaveDirectory_.c_str());
    throw std::invalid_argument("RGB save directory is not a directory. NOTE: relative paths and \'~\' may not work as you think in ROS\n\n");
  }

  if (dirExists(irSaveDirectory_.c_str()) <= 0)
  {
    ROS_FATAL("Save Directory %s invalid", irSaveDirectory_.c_str());
    throw std::invalid_argument("IR save directory is not a directory. NOTE: relative paths and \'~\' may not work as you think in ROS\n\n");
  }
  
  if (dirExists(temperatureSaveDirectory_.c_str()) <= 0)
  {
    ROS_FATAL("Save Directory %s invalid", temperatureSaveDirectory_.c_str());
    throw std::invalid_argument("Temperature save directory is not a directory. NOTE: relative paths and \'~\' may not work as you think in ROS\n\n");
  }

  ROS_WARN("Saving images to\n\tRGB - %s\n\tIR - %s\n\tTemp - %s", 
    rgbSaveDirectory_.c_str(), irSaveDirectory_.c_str(), temperatureSaveDirectory_.c_str());

  frameCount_ = 0;

  srand (time(NULL));
  runID_ = rand() % 10000000;

  //subscribers
  image_transport::ImageTransport it(nh_);
  displayImageSub_ = it.subscribe("seek_camera/displayImage", 10, &DragoonImageCapture::displayDataCallback, this);
  temperatureImageSub_ = it.subscribe("seek_camera/temperatureImageCelcius", 10, &DragoonImageCapture::temperatureDataCallback, this);
  rgbImageSub_ = it.subscribe("camera/color/image_raw", 10, &DragoonImageCapture::rgbDataCallback, this);

  captureCommandSub_ = nh_.subscribe("dragoonCaptureCommand", 10, &DragoonImageCapture::captureCommandCallback, this);

  capturedDisplayImagePub_ = it.advertise("dragoon_image_capture/displayImage", 10);
  capturedRGBImagePub_ = it.advertise("dragoon_image_capture/rgbImage", 10);
  capturedTemperatureImagePub_ = it.advertise("dragoon_image_capture/temperatureImageCelcius", 10);

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "dragoon_image_capture");
  DragoonImageCapture node;
  while (ros::ok())
    ros::spinOnce();
  return 0;
}

// Copyright 2018 Chen, Zhuhui
//
// Permission is hereby granted, free of charge, to any person
// obtaining a copy of this software and associated documentation
// files (the "Software"), to deal in the Software without
// restriction, including without limitation the rights to use, copy,
// modify, merge, publish, distribute, sublicense, and/or sell copies
// of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
// BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
// ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
// CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


// Provides a node giving out video stream, which should be handful
// for debuging purpose.
//
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/videoio.hpp>
#include <camera_info_manager/camera_info_manager.h>
#include <string>

int main(int argc, char* argv[]) {

  ros::init(argc, argv, "file_stream");
  ros::NodeHandle nh("~");

  std::string file;
  bool repeat;
  int fps;

  nh.param("file", file, std::string(""));
  nh.param("repeat", repeat, true);
  nh.param("fps", fps, 60);

  std::string camera_info_url;
  std::string camera_name;
  
  nh.param("camera_info_url", camera_info_url, std::string("package://aruco_track/resources/camera_info.yaml"));
  nh.param("camera_name", camera_name, std::string("camera"));

  camera_info_manager::CameraInfoManager camera_info_mgr(nh, camera_name, camera_info_url);

  sensor_msgs::CameraInfo cam_info;
  if (!camera_info_mgr.loadCameraInfo(camera_info_url)) {
    ROS_WARN("Failed to load camera settings at %s.", camera_info_url.c_str());
  } else {
    ROS_INFO("Camera settings at %s successfully loaded", camera_info_url.c_str());
    cam_info = camera_info_mgr.getCameraInfo();
  }

  if (file == "") {
    ROS_ERROR("Please specify the source file stream with `file'");
    return -1;
  }

  ROS_INFO("Using (%s) as the file stream source. %s",
	   file.c_str(),
	   (repeat ? "(repeat)" : "(non-repeat)"));

  ros::Rate loop_rate(fps);

  cv::VideoCapture cap(file);
  cv::Mat frame;
  sensor_msgs::ImagePtr msg;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher image_pub = it.advertise("camera/image", 1);

  ros::Publisher camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1, true);

  cam_info.header.seq = 0;
  cam_info.header.frame_id = std::string("");
  cam_info.header.stamp = ros::Time::now();
  camera_info_pub.publish(cam_info);
    
  while (nh.ok()) {
    cap >> frame;
    if (frame.empty()) {
      cap.release();
      if (repeat) {
	ROS_INFO("Has reached the end of the source stream. Repeating.");
	cap = cv::VideoCapture(file);
	cap >> frame;
      } else {
	ROS_INFO("Has reached the end of the source stream. Exiting.");
	break;
      }
    }

    if (frame.empty()) {
      // continue looping if the frame is still empty.
      continue;
    }

    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    image_pub.publish(msg);
    
    loop_rate.sleep();
  }

  return 0;
}

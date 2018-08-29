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

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include "board_estimation.h"
#include "settings.h"

static aruco_track::Settings settings;

static ros::Publisher* p_debug_img_pub = nullptr;

void image_callback(const sensor_msgs::ImageConstPtr& msg) {
  if (!settings.camera_info_updated()) {
    ROS_INFO("Waiting for CameraInfo message");
    return;
  }
  cv_bridge::CvImageConstPtr img_ptr;
  try {
    img_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat rvec;
  cv::Mat tvec;
  
  bool processed;
  if (settings.publish_debug_image()) {
    cv::Mat undistorted;
    processed = aruco_track::ProcessFrame(img_ptr->image, settings, rvec, tvec, undistorted);
    
    if (processed) {
      aruco_track::DrawAxisOnImage(undistorted,
				   settings,
				   rvec,
				   tvec);
    
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", undistorted).toImageMsg();
      p_debug_img_pub->publish(msg);
    }
    
  } else {
    
    processed = aruco_track::ProcessFrame(img_ptr->image, settings, rvec, tvec);
    
  }
  
  if (processed) {
    aruco_track::BroadcastCameraTransform(rvec, tvec);
  }
}

void camera_info_callback(const sensor_msgs::CameraInfoConstPtr& msg) {
  ROS_INFO("Got CameraInfo message. Updating Settings object.");
  settings.UpdateWithCameraInfoMsg(msg);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "aruco_track");
  ros::NodeHandle nh("~");
  
  int markers_dict;
  int board_num_x;
  int board_num_y;
  float board_marker_length;
  float board_marker_separation;
  bool publish_debug_image;
  
  nh.param("markers_dict", markers_dict, -1);
  nh.param("board_num_x", board_num_x, -1);
  nh.param("board_num_y", board_num_y, -1);
  nh.param("board_marker_length", board_marker_length, -1.f);
  nh.param("board_marker_separation", board_marker_separation, -1.f);
  nh.param("publish_debug_image", publish_debug_image, false);

  if (markers_dict < 0) {
    ROS_ERROR("markers_dict not set properly.");
    return -1;
  }
  if (board_num_x < 0) {
    ROS_ERROR("board_num_x not set properly.");
    return -1;
  }
  if (board_num_y < 0) {
    ROS_ERROR("board_num_y not set properly.");
    return -1;
  }
  if (board_marker_length < 0) {
    ROS_ERROR("board_marker_length not set properly.");
    return -1;
  }
  if (board_marker_separation < 0) {
    ROS_ERROR("board_marker_separation not set properly.");
    return -1;
  }
  
  settings.UpdateArucoParameters(markers_dict,
				 board_num_x, board_num_y,
				 board_marker_length, board_marker_separation);
  settings.set_publish_debug_image(publish_debug_image);

  ros::Subscriber source_sub = nh.subscribe("source", 1, image_callback);
  ros::Subscriber cam_info_sub = nh.subscribe("camera_info", 1, camera_info_callback);
  ros::Publisher debug_img_pub;
  if (publish_debug_image) {
    debug_img_pub = nh.advertise<sensor_msgs::Image>("debug_image", 1);
    p_debug_img_pub = &debug_img_pub;
  }

  while (nh.ok()) {
    ros::spin();
  }
  
}

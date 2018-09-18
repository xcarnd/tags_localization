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
#ifndef _ARUCO_TRACK_ESTIMATOR_HPP_
#define _ARUCO_TRACK_ESTIMATOR_HPP_

#include <memory>

#include <opencv2/core.hpp>

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <aruco_track/SetHomePosition.h>

#include "settings.hpp"

namespace aruco_track {

  class _SetHomePositionHelper;

  class BoardEstimator {
  private:
    bool home_set_;
    const Settings& settings_;
    
    ros::NodeHandle& node_handle_;
    ros::Publisher debug_img_pub_;

    std::shared_ptr<_SetHomePositionHelper> helper_;

    ros::Publisher pose_publisher_;

    ros::Subscriber source_sub_;
    ros::Subscriber set_home_position_sub_;

    tf2_ros::StaticTransformBroadcaster static_transform_broadcaster_;
  public:
    BoardEstimator(ros::NodeHandle& node_handle, const Settings& settings);
    
    bool ProcessFrame(const cv::InputArray& frame,
		      cv::Mat& rvec, cv::Mat& tvec,
		      cv::OutputArray& undistorted = cv::noArray());

    void DrawAxisOnImage(cv::InputOutputArray& image,
			 const cv::Mat& rvec, const cv::Mat& tvec);
  
    void InitSubscribers();

    // callback handlers
    void HandleSetHomePosition(const SetHomePositionConstPtr& msg);
    void HandleImage(const sensor_msgs::ImageConstPtr& msg);

  private:
    geometry_msgs::PoseStamped EstimatePose(const cv::Mat& rvec, const cv::Mat& tvec);

  };
  
}

#endif

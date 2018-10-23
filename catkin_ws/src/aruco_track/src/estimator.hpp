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
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include "settings.hpp"

namespace aruco_track {

  class BoardEstimator {
  private:
    const Settings& settings_;
    
    ros::NodeHandle& node_handle_;
    ros::NodeHandle& parent_node_handle_;

    ros::Publisher board_pose_pub_;
    ros::Publisher estimated_pose_pub_;

    ros::Subscriber source_sub_;

    // subcriber for pose reported from mavros
    ros::Subscriber fcu_pose_sub_;
    // and also the board pose
    ros::Subscriber board_pose_sub_;

    // tf broadcaster
    tf2_ros::StaticTransformBroadcaster static_transform_broadcaster_;
    tf2_ros::TransformBroadcaster transform_broadcaster_;

    // tf listener & filter
    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_listener_;
    message_filters::Subscriber<geometry_msgs::PoseStamped> board_pose_filter_;
    tf2_ros::MessageFilter<geometry_msgs::PoseStamped> tf2_filter_;
  public:
    BoardEstimator(ros::NodeHandle& nh,
		   ros::NodeHandle& parent_nh,
		   const Settings& settings);
    
    bool ProcessFrame(const cv::InputArray& frame,
		      cv::Mat& rvec, cv::Mat& tvec,
		      cv::OutputArray& undistorted = cv::noArray());

    void DrawAxisOnImage(cv::InputOutputArray& image,
			 const cv::Mat& rvec, const cv::Mat& tvec);
  
    void Init();

    // subcribers
    void HandleImage(const sensor_msgs::ImageConstPtr& msg);
    void HandleFcuPose(const geometry_msgs::PoseStampedConstPtr& msg);
    void HandleBoardPose(const geometry_msgs::PoseStampedConstPtr& msg);
  private:
    void EstimatePose(const cv::Mat& rvec, const cv::Mat& tvec);

    void EstimateAndPublishPosition(const geometry_msgs::PoseStampedConstPtr& msg);

  };
  
}

#endif

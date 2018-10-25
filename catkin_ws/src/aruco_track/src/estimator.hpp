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

#include <opencv2/core.hpp>

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include "board.hpp"
#include "base_node.hpp"

namespace aruco_track {

  class BoardEstimator : public RunnableNode {
  private:
    // board parameters
    Board board_;

    // parameters for performing undistortion.
    cv::Mat camera_matrix_;
    cv::Mat distort_coeffs_;
    bool camera_info_ready_;

    // estimated camera pose w.r.t. to board frame
    ros::Publisher camera_pose_pub_;

    // estimated pose w.r.t. to map frame
    ros::Publisher estimated_pose_pub_;

    // subscriber for the camera captured image
    ros::Subscriber source_sub_;

    // subscriber for camera info
    ros::Subscriber camera_info_sub_;

    // tf broadcaster
    tf2_ros::StaticTransformBroadcaster static_transform_broadcaster_;
    tf2_ros::TransformBroadcaster transform_broadcaster_;

    // tf listener & filter
    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_listener_;
    message_filters::Subscriber<geometry_msgs::PoseStamped> filter_sub_;
    tf2_ros::MessageFilter<geometry_msgs::PoseStamped> tf2_filter_;
  public:
    BoardEstimator(int argc, char* argv[], const std::string& node_name);
    
    bool ProcessFrame(const cv::InputArray& frame,
		      cv::Mat& rvec, cv::Mat& tvec,
		      cv::OutputArray& undistorted = cv::noArray());

    void DrawAxisOnImage(cv::InputOutputArray& image,
			 const cv::Mat& rvec, const cv::Mat& tvec);
  
    void Init();

    // subcriber callback
    void HandleImage(const sensor_msgs::ImageConstPtr& msg);
    void HandleCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg);
  private:
    void EstimatePose(const cv::Mat& rvec, const cv::Mat& tvec);

    void EstimateAndPublishPosition(const geometry_msgs::PoseStampedConstPtr& msg);

  };
  
}

#endif

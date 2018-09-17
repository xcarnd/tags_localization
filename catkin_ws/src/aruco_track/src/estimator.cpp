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

//
// Detect, estimate pose from board configuration and broadcast
// tf2 messages.

#include <vector>
#include <memory>
#include <iostream>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include <Eigen/Geometry>

#include "estimator.h"
#include "frame_def.h"

using namespace std;
using namespace cv;
using namespace cv::aruco;

namespace aruco_track {

  class _SetHomePositionHelper {
    friend class BoardEstimator;
  private:
    ros::Subscriber pose_from_board_sub_;
    ros::Subscriber pose_from_map_sub_;
    
    bool last_pose_board_set_;
    bool last_pose_map_set_;

    ros::NodeHandle& nh_;

  private:
    void HandlePoseFromBoard(const geometry_msgs::PoseStampedConstPtr& msg) {
      last_pose_board = *msg;
      last_pose_board_set_ = true;
      
      Join();
    }

    void HandlePoseFromMap(const geometry_msgs::PoseStampedConstPtr& msg) {
      last_pose_map = *msg;
      last_pose_map_set_ = true;
      
      Join();
    }

    void Join() {
      if (last_pose_board_set_ && last_pose_map_set_) {
        ROS_INFO("Pose from board and pose from map set.");
        pose_from_board_sub_.shutdown();
        pose_from_map_sub_.shutdown();
      }
    }
    
  public:
    geometry_msgs::PoseStamped last_pose_board;
    geometry_msgs::PoseStamped last_pose_map;

    _SetHomePositionHelper(ros::NodeHandle& nh)
      : last_pose_board_set_(false), last_pose_map_set_(false), nh_(nh) {}

    void ListenForPoses() {
      last_pose_board_set_ = false;
      last_pose_map_set_ = false;
      pose_from_board_sub_ = nh_.subscribe("/aruco_track/pose", 1,
					   &_SetHomePositionHelper::HandlePoseFromBoard, this);
      pose_from_map_sub_ = nh_.subscribe("/marvos/local_position/pose", 1,
					 &_SetHomePositionHelper::HandlePoseFromMap, this);
      ROS_INFO("Subscribed to /aruco_track/pose & /mavros_local_position/pose");
    }
  };

  BoardEstimator::BoardEstimator(ros::NodeHandle& node_handle, const Settings& settings)
    : home_set_(false),
      settings_(settings),
      node_handle_(node_handle) {
    debug_img_pub_ = node_handle_.advertise<sensor_msgs::Image>("debug_image", 1);
    pose_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>("pose", 1);
  }

  void BoardEstimator::HandleImage(const sensor_msgs::ImageConstPtr& msg) {
    if (!settings_.camera_info_updated()) {
      ROS_INFO("Waiting for CameraInfo message.");
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
    if (settings_.publish_debug_image()) {
      cv::Mat undistorted;
      processed = this->ProcessFrame(img_ptr->image, rvec, tvec, undistorted);
    
      if (processed) {
      	this->DrawAxisOnImage(undistorted, rvec, tvec);
    
	      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", undistorted).toImageMsg();
	      debug_img_pub_.publish(msg);
      }
    
    } else {
    
      processed = this->ProcessFrame(img_ptr->image, rvec, tvec);
    
    }

    if (processed) {
      geometry_msgs::PoseStamped pose = this->EstimatePose(rvec, tvec);
      pose_publisher_.publish(pose);
    }
  }

  void BoardEstimator::HandleSetHomePosition(const SetHomePositionConstPtr& msg) {
    // when starts setting home position, the estimator will first collect enough state estimations,
    // both from board estimation and map local position.
    //
    // then the average of the collection information is calculated. the average will then be used
    // as a reference as home position.
    home_set_ = false;
    helper_ = std::make_shared<_SetHomePositionHelper>(node_handle_);
    helper_->ListenForPoses();
  }

  bool BoardEstimator::ProcessFrame(const cv::InputArray& frame,
				    cv::Mat& rvec, cv::Mat& tvec,
				    cv::OutputArray& undistorted) {
    cv::Mat helper;
    
    // processing step:
    // 1. undistort frame image
    if (undistorted.needed()) {
      undistort(frame, undistorted, settings_.camera_matrix(), settings_.distort_coeffs());
    } else {
      undistort(frame, helper, settings_.camera_matrix(), settings_.distort_coeffs());      
    }
  
    // 2. detect aruco markers
    vector<int> ids;
    vector<vector<Point2f> > corners;
    if (undistorted.needed()) {
      detectMarkers(undistorted, settings_.markers_dict(), corners, ids);
    } else {
      detectMarkers(helper, settings_.markers_dict(), corners, ids);
    }

    if (ids.size() == 0) {
      // no markers detected. skip frame.
      return false;
    }
  
    // 3. estimate pose
    int num_used = estimatePoseBoard(corners, ids, settings_.board(),
				     settings_.camera_matrix(),
				     settings_.distort_coeffs(),
				     rvec,
				     tvec);

    if (num_used == 0) {
      // pose not estimated
      return false;
    }

    return true;
  }

  void BoardEstimator::DrawAxisOnImage(cv::InputOutputArray& image,
				       const::Mat& rvec, const cv::Mat& tvec) {
    cv::aruco::drawAxis(image,
			settings_.camera_matrix(),
			settings_.distort_coeffs(),
			rvec, tvec,
			settings_.board_marker_length());
  }

  /**
   * Broadcast dynamic transform between the board coordindate and the 
   * camera coordinate.
   */
  geometry_msgs::PoseStamped BoardEstimator::EstimatePose(const cv::Mat& rvec, const cv::Mat& tvec) {
    static cv::Mat rot_mat;

    cv::Rodrigues(rvec, rot_mat);
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > mat(rot_mat.ptr<double>());

    Eigen::Quaternion<double> q(mat);

    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = FRAME_BOARD;

    msg.pose.position.x = tvec.at<double>(0, 0);
    msg.pose.position.y = tvec.at<double>(1, 0);
    msg.pose.position.z = tvec.at<double>(2, 0);
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();

    return msg;
  }

  void BoardEstimator::InitSubscribers() {
    // listening for image
    source_sub_ =
      node_handle_.subscribe("source", 1,
			     &BoardEstimator::HandleImage, this);
    ROS_INFO("Listening for /aruco_track/source topic.");
    
    // listening for request of setting "home position"
    set_home_position_sub_ =
      node_handle_.subscribe("set_home_position", 1,
			     &BoardEstimator::HandleSetHomePosition, this);
    ROS_INFO("Listening for /aruco_track/set_home_position topic.");
  
  }

}

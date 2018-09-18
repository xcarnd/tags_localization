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
#include <geometry_msgs/TransformStamped.h>
#include <tf2/utils.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include <Eigen/Geometry>

#include "estimator.hpp"
#include "frame_def.hpp"

using namespace std;
using namespace cv;
using namespace cv::aruco;

namespace aruco_track {

  class _SetHomePositionHelper {
    friend class BoardEstimator;
  private:
    geometry_msgs::PoseStamped last_pose_board_;
    geometry_msgs::PoseStamped last_pose_map_;
    ros::Subscriber pose_from_board_sub_;
    ros::Subscriber pose_from_map_sub_;
    
    bool last_pose_board_set_;
    bool last_pose_map_set_;

    ros::NodeHandle& nh_;

    tf2_ros::StaticTransformBroadcaster& static_transform_broadcaster_;

  private:
    void HandlePoseFromBoard(const geometry_msgs::PoseStampedConstPtr& msg) {
      last_pose_board_ = *msg;
      last_pose_board_set_ = true;
      
      Join();
    }

    void HandlePoseFromMap(const geometry_msgs::PoseStampedConstPtr& msg) {
      last_pose_map_ = *msg;
      last_pose_map_set_ = true;
      
      Join();
    }

    void Join() {
      if (last_pose_board_set_ && last_pose_map_set_) {
        ROS_INFO("Pose from board and pose from map set.");
        // define the origin of map frame to be somewhere with xyz = 0 and rpy = 0
        // define the origin of board frame to be the lower left corner of the tag board, z point outwards.
        //
        // pose board contains transformation information for camera pose in board frame.
        //
        // pose map contains transformation information for fcu pose in map frame.
        //
        // and now we want to solve this problem: given a the camera pose in board frame, what would 
        // the pose of the fcu in map frame?
        //
        // here is my solution:
        // when set_home_position message is received, define the last camera pose as the origin
        // of the frame 'camera_base'. we then immediately can establish a static tf transform between 
        // the board frame and camera_base frame.
        //
        // suppose the transform from camera_base frame to map frame is T_c_m. given a pose P in camera_base frame,
        // we can transform it into map frame by appling T_c_m on P. if we apply T_c_m on the origin point
        // of camera_base frame, we'll get the pose for camera in map frame when we received the last pose board
        // message.
        //
        // and now we have pose of camera in map frame and pose of fcu in map frame. Their centers won't
        // coincide so their reading won't be the same, but the transformation between them is fixed 
        // (noted as T_cam_fcu) and won't changed over time.
        //
        // although we might not solve T_c_m and T_cam_fcu directly, but since they're all fixed transform, 
        // we can combine them as a whole, i.e, T_cam2fcu = T_cam_fcu * T_c_m. This is a transform from pose of 
        // camera in camera_base frame to pose of fcu in map frame. This transform is solvable because we have 
        // a pair of known input and output: the origin of the camera input and the pose of fcu in map.
        //
        // so, together with transform from board frame to camera_base frame, we can now transform any 
        // camera pose in board to fcu in map.
        std::vector<geometry_msgs::TransformStamped> static_tfs;
	ros::Time stamp = ros::Time::now();

        // since we know the pose of camera_base in board, the same pose can be used as the transform from camera_base to board.
        geometry_msgs::TransformStamped tf_board_camera_base;
        tf_board_camera_base.header.stamp = stamp;
        tf_board_camera_base.header.frame_id = "camera_base";
        tf_board_camera_base.child_frame_id = "board";
        tf_board_camera_base.transform.translation.x = last_pose_board_.pose.position.x;
        tf_board_camera_base.transform.translation.y = last_pose_board_.pose.position.y;
        tf_board_camera_base.transform.translation.z = last_pose_board_.pose.position.y;
        tf_board_camera_base.transform.rotation.x = last_pose_board_.pose.orientation.x;
        tf_board_camera_base.transform.rotation.y = last_pose_board_.pose.orientation.y;
        tf_board_camera_base.transform.rotation.z = last_pose_board_.pose.orientation.z;
        tf_board_camera_base.transform.rotation.w = last_pose_board_.pose.orientation.w;

        static_tfs.push_back(tf_board_camera_base);

        // and the transform from pose in camera_base to fcu in map.
        geometry_msgs::TransformStamped tf_camera_base_map;
        tf_camera_base_map.header.stamp = stamp;
        tf_camera_base_map.header.frame_id = "camera_base";
        tf_camera_base_map.child_frame_id = "map";
        tf_camera_base_map.transform.translation.x = last_pose_map_.pose.position.x;
        tf_camera_base_map.transform.translation.y = last_pose_map_.pose.position.y;
        tf_camera_base_map.transform.translation.z = last_pose_map_.pose.position.z;
        tf_camera_base_map.transform.rotation.x = last_pose_map_.pose.orientation.x;
        tf_camera_base_map.transform.rotation.y = last_pose_map_.pose.orientation.y;
        tf_camera_base_map.transform.rotation.z = last_pose_map_.pose.orientation.z;
        tf_camera_base_map.transform.rotation.w = last_pose_map_.pose.orientation.w;

        static_tfs.push_back(tf_camera_base_map);

        static_transform_broadcaster_.sendTransform(static_tfs);

        pose_from_board_sub_.shutdown();
        pose_from_map_sub_.shutdown();
      }
    }
    
  public:
    _SetHomePositionHelper(ros::NodeHandle& nh, tf2_ros::StaticTransformBroadcaster& static_transform_broadcaster_)
      : last_pose_board_set_(false), last_pose_map_set_(false), nh_(nh), static_transform_broadcaster_(static_transform_broadcaster_) {}

    void ListenForPoses() {
      last_pose_board_set_ = false;
      last_pose_map_set_ = false;
      pose_from_board_sub_ = nh_.subscribe("board_pose", 1,
					   &_SetHomePositionHelper::HandlePoseFromBoard, this);
      pose_from_map_sub_ = nh_.subscribe("reference_pose", 1,
					 &_SetHomePositionHelper::HandlePoseFromMap, this);
      ROS_INFO("Subscribed to /aruco_track/pose & /aruco_track/reference_pose");
    }
  };

  BoardEstimator::BoardEstimator(ros::NodeHandle& node_handle, const Settings& settings)
    : home_set_(false),
      settings_(settings),
      node_handle_(node_handle) {
    debug_img_pub_ = node_handle_.advertise<sensor_msgs::Image>("debug_image", 1);
    pose_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>("board_pose", 1);
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
    helper_ = std::make_shared<_SetHomePositionHelper>(node_handle_, static_transform_broadcaster_);
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

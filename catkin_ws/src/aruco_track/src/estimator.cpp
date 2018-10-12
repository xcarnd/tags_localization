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

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include <vector>
#include <memory>
#include <sstream>
#include <cmath>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/utils.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include <Eigen/Geometry>

#include "estimator.hpp"
#include "frame_def.hpp"

using namespace std;
using namespace cv;
using namespace cv::aruco;

namespace {
  inline void fillInverseIntoMsg(const geometry_msgs::Quaternion& quaternion, const geometry_msgs::Point& point, geometry_msgs::TransformStamped& msg) {
    tf2::Transform from(tf2::Quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w),
			tf2::Vector3(point.x, point.y, point.z));
    const tf2::Transform& inverse = from.inverse();
    msg.transform.translation.x = inverse.getOrigin().getX();
    msg.transform.translation.y = inverse.getOrigin().getY();
    msg.transform.translation.z = inverse.getOrigin().getZ();
    msg.transform.rotation.x = inverse.getRotation().getX();
    msg.transform.rotation.y = inverse.getRotation().getY();
    msg.transform.rotation.z = inverse.getRotation().getZ();
    msg.transform.rotation.w = inverse.getRotation().getW();
  }

  inline double deg2rad(double deg) {
    return deg * M_PI / 180;
  }
}

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
      ROS_DEBUG("Pose from board received");
      last_pose_board_ = *msg;
      last_pose_board_set_ = true;
      
      Join();
    }

    void HandlePoseFromMap(const geometry_msgs::PoseStampedConstPtr& msg) {
      ROS_DEBUG("Pose from map received");
      last_pose_map_ = *msg;
      last_pose_map_set_ = true;
      
      Join();
    }

    void Join() {
      if (last_pose_board_set_ && last_pose_map_set_) {
        ROS_INFO("Pose from board and pose from map set.");
	
        std::vector<geometry_msgs::TransformStamped> static_tfs;
	ros::Time stamp = ros::Time::now();

	// to build up the tf tree for estimation pose, here're what we have:
	// 1. we have last_pose_board_ which gives us tf camera_base -> board
	// 2. we have last_pose_map_ which gives us map -> fcu_base
	// 3. we are passing in fcu -> camera from the parameter server.
	// 4. fcu_base -> camera_base is just the same as fcu -> camera
	// 5. every time pose board is estimated, we will have camera -> board
	//
	// so we can build up such a tf tree:
	// fcu -> -> camera -> board -> camera_base -> fcu_base -> map
	//
	// all these relations are fixed except for board -> camera, which has to be
	// dynamically update every time we get camera -> board.
	//
        // as the document of aruco_track says, last_pose_board_ is actually the transformation from
	// board coordinate system to camera_base system, so we have camera_base -> board.
	//
	// yet we need the inverse: board -> camera_base
	//
        geometry_msgs::TransformStamped tf_board_to_camera_base;
        tf_board_to_camera_base.header.stamp = stamp;
        tf_board_to_camera_base.header.frame_id = "board";
        tf_board_to_camera_base.child_frame_id = "camera_base";
	fillInverseIntoMsg(last_pose_board_.pose.orientation,
			   last_pose_board_.pose.position,
			   tf_board_to_camera_base);
	
        static_tfs.push_back(tf_board_to_camera_base);

	// fcu -> camera and camera_base -> fcu_base this is specified outside the code.
	// error of this transform can be considered as system bias.
	// notice that fcu_base is specified in FRD manner.
	std::string tf_fcu2cam;
	if (!nh_.getParam("tf_fcu2cam", tf_fcu2cam)) {
	  ROS_WARN("tf_fcu2cam not set. Using '0 0 0 0 0 0' as the default value.");
	  tf_fcu2cam = "0 0 0 0 0 0";
	}

	std::istringstream iss(tf_fcu2cam);
	double tx, ty, tz, r, p, y;
	iss>>tx>>ty>>tz>>r>>p>>y;
	
	tf2::Quaternion q_helper;
	q_helper.setRPY(deg2rad(r), deg2rad(p), deg2rad(y));
	double qx = q_helper.getX();
	double qy = q_helper.getY();
	double qz = q_helper.getZ();
	double qw = q_helper.getW();

        geometry_msgs::TransformStamped tf_fcu_to_camera;
        tf_fcu_to_camera.header.stamp = stamp;
        tf_fcu_to_camera.header.frame_id = "fcu";
        tf_fcu_to_camera.child_frame_id = "camera";
        tf_fcu_to_camera.transform.translation.x = tx;
        tf_fcu_to_camera.transform.translation.y = ty;
        tf_fcu_to_camera.transform.translation.z = tz;
        tf_fcu_to_camera.transform.rotation.x = qx;
        tf_fcu_to_camera.transform.rotation.y = qy;
        tf_fcu_to_camera.transform.rotation.z = qz;
        tf_fcu_to_camera.transform.rotation.w = qw;

        static_tfs.push_back(tf_fcu_to_camera);

	// camera_base -> fcu_base is just the inverse
        geometry_msgs::TransformStamped tf_base_camera_to_fcu;
        tf_base_camera_to_fcu.header.stamp = stamp;
        tf_base_camera_to_fcu.header.frame_id = "camera_base";
        tf_base_camera_to_fcu.child_frame_id = "fcu_base";
	geometry_msgs::Quaternion quaternion;
	quaternion.x = qx;
	quaternion.y = qy;
	quaternion.z = qz;
	quaternion.w = qw;
	geometry_msgs::Point point;
	point.x = tx;
	point.y = ty;
	point.z = tz;
	fillInverseIntoMsg(quaternion, point, tf_base_camera_to_fcu);

        static_tfs.push_back(tf_base_camera_to_fcu);

        // last_pose_map_ is the pose of fcu in map frame. so we have a transform from map to fcu_base
	// but we need the inverse
        geometry_msgs::TransformStamped tf_fcu_base_to_map;
        tf_fcu_base_to_map.header.stamp = stamp;
        tf_fcu_base_to_map.header.frame_id = "fcu_base";
        tf_fcu_base_to_map.child_frame_id = "map";
	fillInverseIntoMsg(last_pose_map_.pose.orientation,
			   last_pose_map_.pose.position,
			   tf_fcu_base_to_map);

        static_tfs.push_back(tf_fcu_base_to_map);

        static_transform_broadcaster_.sendTransform(static_tfs);

	ROS_INFO("Fixed frame references established and broadcasted.");

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
      ROS_INFO("Subscribed to aruco_track/board_pose & aruco_track/reference_pose");
    }
  };

  BoardEstimator::BoardEstimator(ros::NodeHandle& node_handle, const Settings& settings)
    : home_set_(false),
      settings_(settings),
      node_handle_(node_handle) {
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
  
    if (this->ProcessFrame(img_ptr->image, rvec, tvec)) {
      geometry_msgs::PoseStamped pose = this->EstimatePose(rvec, tvec);
      pose_publisher_.publish(pose);

      // also broadcast dynamic transform camera -> board.
      // this is the inverse of pose
      geometry_msgs::TransformStamped tf_camera_to_board;
      tf_camera_to_board.header.stamp = ros::Time::now();
      tf_camera_to_board.header.frame_id = FRAME_CAMERA;
      tf_camera_to_board.child_frame_id = FRAME_BOARD;
      tf_camera_to_board.transform.translation.x = pose.pose.position.x;
      tf_camera_to_board.transform.translation.y = pose.pose.position.y;
      tf_camera_to_board.transform.translation.z = pose.pose.position.z;
      tf_camera_to_board.transform.rotation.x = pose.pose.orientation.x;
      tf_camera_to_board.transform.rotation.y = pose.pose.orientation.y;
      tf_camera_to_board.transform.rotation.z = pose.pose.orientation.z;
      tf_camera_to_board.transform.rotation.w = pose.pose.orientation.w;
    
      transform_broadcaster_.sendTransform(tf_camera_to_board);
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
    msg.header.frame_id = FRAME_CAMERA;

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

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
#include <sstream>

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

#include "utils.hpp"
#include "estimator.hpp"
#include "frame_def.hpp"

using namespace std;
using namespace cv;
using namespace cv::aruco;


namespace aruco_track {

  BoardEstimator::BoardEstimator(ros::NodeHandle& nh,
				 ros::NodeHandle& parent_nh,
				 const Settings& settings)
    : settings_(settings),
      node_handle_(nh),
      parent_node_handle_(parent_nh),
      tf2_listener_(tf2_buffer_) {
    estimated_pose_pub_ = node_handle_.advertise<geometry_msgs::PoseStamped>("estimated_pose", 1);
  }

  void BoardEstimator::HandleFcuPose(const geometry_msgs::PoseStampedConstPtr& msg) {
    // tf tree:
    // map -> fcu_flu -> fcu -> camera -> board -> board_center
    //
    // what we want to know is the rotational transformation from map to board_center
    //
    // static tf:
    // fcu -> camera
    // board -> board_center
    //
    // dynamic rotation:
    // map -> fcu_flu
    // fcu_flu -> fcu
    // camera -> board

    // in this msg handler, we will publish map -> fcu_flu and fcu_flu -> fcu

    // map -> fcu_flu can be broken into two part: position, which
    // is contained in the position part of msg. orientation, which is a
    // fixed quaternion.
    auto msg_map_to_fcu_flu =
      makeTransformStamped("map", "fcu_flu",
    			   msg->pose.position,
    			   makeTf2QuaternionFromRPYDegree(0, 0, 90));
    transform_broadcaster_.sendTransform(msg_map_to_fcu_flu);

    // fcu_flu -> fcu is just the orientation part of msg
    auto msg_fcu_flu_to_fcu =
      makeTransformStamped("fcu_flu", "fcu",
    			   0, 0, 0,
    			   msg->pose.orientation);

    transform_broadcaster_.sendTransform(msg_fcu_flu_to_fcu);
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
      this->EstimatePose(rvec, tvec);
    }
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
  void BoardEstimator::EstimatePose(const cv::Mat& rvec, const cv::Mat& tvec) {
    static cv::Mat rot_mat;

    cv::Rodrigues(rvec, rot_mat);
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > mat(rot_mat.ptr<double>());

    Eigen::Quaternion<double> q(mat);

    // as the document says, rvec + tvec is the transformation from points
    // in board frame to camera frame.
    //
    // in other words, this is how to get camera -> board tf transform.
    auto msg = makeTransformStamped("camera", "board",
				    tvec.at<double>(0, 0),
				    tvec.at<double>(1, 0),
				    tvec.at<double>(2, 0),
				    q);
    transform_broadcaster_.sendTransform(msg);
    //    this->EstimateAndPublishPosition();
  }

  void BoardEstimator::EstimateAndPublishPosition() {
    // once the tf tree:
    //   map -> fcu_flu -> fcu -> camera -> board -> board_center
    // has built up, we will know how to rotate map frame to make it
    // aligning with board_center
    //
    // we then know how to transform points in board_center to map.
    // we will suppose the origin of map will coincide with board_center,
    // so we only need to care about the rotation part of transform.

    // so what is the center of the camera in board_center frame?
    geometry_msgs::TransformStamped tf0 =
      tf2_buffer_.lookupTransform("camera", "board_center", ros::Time(0));
    geometry_msgs::PointStamped pt_in;
    pt_in.header.frame_id = "camera";
    pt_in.header.stamp = ros::Time::now();
    pt_in.point.x = 0;
    pt_in.point.y = 0;
    pt_in.point.z = 0;
    geometry_msgs::PointStamped pt_out;
    tf2::doTransform(pt_in, pt_out, tf0);
    
    // what if we rotate that point in board_center into map? notice that we only
    // need rotation here, so we will throw away any positional information in the
    // transformation
    geometry_msgs::TransformStamped tf =
      tf2_buffer_.lookupTransform("map", "board_center", ros::Time(0));
    tf.transform.translation.x = 0;
    tf.transform.translation.y = 0;
    tf.transform.translation.z = 0;
    geometry_msgs::PointStamped pt_final;
    tf2::doTransform(pt_out, pt_final, tf);

    // finally, we can publish that out!
    geometry_msgs::PoseStamped estimated_pose;
    estimated_pose.header.stamp = ros::Time::now();
    estimated_pose.header.frame_id = "map";
    estimated_pose.pose.position = pt_final.point;
    estimated_pose.pose.orientation.x = 0;
    estimated_pose.pose.orientation.y = 0;
    estimated_pose.pose.orientation.z = 0;
    estimated_pose.pose.orientation.w = 1;
    
    this->estimated_pose_pub_.publish(estimated_pose);
  }

  void BoardEstimator::Init() {
    // listening for image
    source_sub_ =
      node_handle_.subscribe("source", 1,
			     &BoardEstimator::HandleImage, this);
    ROS_INFO("Listening for source topic.");

    pose_sub_ =
      node_handle_.subscribe("pose", 1,
			     &BoardEstimator::HandleFcuPose, this);
    ROS_INFO("Listening for pose topic");

    // setting up static transforms
    // we will have a "board_center" frame, whose origin is at the
    // center of the board frame
    int board_num_x, board_num_y;
    double board_marker_length, board_marker_separation;
    parent_node_handle_.getParam("board_num_x", board_num_x);
    parent_node_handle_.getParam("board_num_y", board_num_y);
    parent_node_handle_.getParam("board_marker_length", board_marker_length);
    parent_node_handle_.getParam("board_marker_separation", board_marker_separation);

    // calculating the size of the board, then get the center.
    double board_width = board_num_x * board_marker_length
      + (board_num_x - 1) * board_marker_separation;
    double board_height = board_num_y * board_marker_length
      + (board_num_y - 1) * board_marker_separation;
    double center_x = board_width / 2;
    double center_y = board_height / 2;

    ROS_INFO("board width: %.2f, board height: %.2f. Creating static transform from board -> board_center at the center of the board.",
	      board_width, board_height);

    std::vector<geometry_msgs::TransformStamped> static_tfs;

    auto tf_board_to_board_center = makeTransformStamped("board", "board_center",
							 center_x, center_y, 0,
							 0, 0, 0, 1);

    static_tfs.push_back(tf_board_to_board_center);

    std::string tf_fcu2cam;
    if (!node_handle_.getParam("tf_fcu2cam", tf_fcu2cam)) {
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

    auto tf_fcu_to_camera =
      makeTransformStamped("fcu", "camera",
			   tx, ty, tz,
			   makeTf2QuaternionFromRPYDegree(r, p, y));

    static_tfs.push_back(tf_fcu_to_camera);

    static_transform_broadcaster_.sendTransform(static_tfs);
  }

}

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

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include <Eigen/Geometry>

#include "utils.hpp"
#include "estimator.hpp"

using namespace std;
using namespace cv;
using namespace cv::aruco;


namespace aruco_track {

  BoardEstimator::BoardEstimator(int argc, char* argv[], const std::string& node_name)
    : RunnableNode(argc, argv, node_name),
      board_(parent_nh_),
      camera_info_ready_(false),
      tf2_listener_(tf2_buffer_),
      tf2_filter_(filter_sub_, tf2_buffer_, "map", 4, nullptr) {
        
    camera_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("camera_pose", 1);
    
    estimated_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("estimated_pose", 1);

    filter_sub_.subscribe(nh_, "camera_pose", 4);

    tf2_filter_.registerCallback(&BoardEstimator::EstimateAndPublishPosition, this);
  }

  void BoardEstimator::HandleCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg) {
    camera_matrix_ = cv::Mat(3, 3, CV_64F);
    distort_coeffs_ = cv::Mat(1, 5, CV_64F);
    
    for (std::size_t i = 0; i < 3; ++i) {
      for (std::size_t j = 0; j < 3; ++j) {
	      camera_matrix_.at<double>(i, j) = msg->K[i * 3 + j];
      }
    }

    for (std::size_t i = 0; i < 5; ++i) {
      distort_coeffs_.at<double>(0, i) = msg->D[i];
    }

    camera_info_ready_ = true;
    ROS_DEBUG("Camera parameters updated.");
  }

  void BoardEstimator::HandleImage(const sensor_msgs::ImageConstPtr& msg) {
    if (!camera_info_ready_) {
      ROS_INFO("Camera parameters not ready. Still waiting for CameraInfo message.");
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
      undistort(frame, undistorted, camera_matrix_, distort_coeffs_);
    } else {
      undistort(frame, helper, camera_matrix_, distort_coeffs_);
    }
  
    // 2. detect aruco markers
    vector<int> ids;
    vector<vector<Point2f> > corners;
    if (undistorted.needed()) {
      detectMarkers(undistorted, board_.markers_dict(), corners, ids);
    } else {
      detectMarkers(helper, board_.markers_dict(), corners, ids);
    }

    if (ids.size() == 0) {
      // no markers detected. skip frame.
      return false;
    }
  
    // 3. estimate pose
    int num_used = estimatePoseBoard(corners, ids, board_.board(),
				     camera_matrix_,
				     distort_coeffs_,
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
			camera_matrix_,
			distort_coeffs_,
			rvec, tvec,
			board_.marker_size());
  }

  /**
   * Broadcast dynamic transform between the board coordindate and the 
   * camera coordinate.
   */
  void BoardEstimator::EstimatePose(const cv::Mat& rvec, const cv::Mat& tvec) {
    static cv::Mat rot_mat;

    cv::Rodrigues(rvec, rot_mat);
    // as mentioned in the document, rvec + tvec is the transformation from
    // points in board frame to camera frame.
    //
    // what we want is the camera pose w.r.t. board frame (and since 
    // board -> board_center is static, we can easily know camera pose
    // w.r.t. board_center frame once we get what we want)
    //
    // so we need the inverse. once we know how to transform points in camera
    // frame to board frame, we can apply that on the origin of the 
    // camera and we will see the pose of the camera w.r.t. board.
    tf2::Transform forward(
      tf2::Matrix3x3{
        rot_mat.at<double>(0, 0), rot_mat.at<double>(0, 1), rot_mat.at<double>(0, 2),
        rot_mat.at<double>(1, 0), rot_mat.at<double>(1, 1), rot_mat.at<double>(1, 2),
        rot_mat.at<double>(2, 0), rot_mat.at<double>(2, 1), rot_mat.at<double>(2, 2)
      },
      tf2::Vector3 {
        tvec.at<double>(0, 0),
        tvec.at<double>(1, 0),
        tvec.at<double>(2, 0)
      });
    
    const tf2::Transform& backward = forward.inverse();
    tf2::Transform in(
      tf2::Quaternion(0, 0, 0, 1),
      tf2::Vector3(0, 0, 0));
    tf2::Transform out = backward * in;

    geometry_msgs::PoseStamped msg;
    tf2::toMsg(out, msg.pose);
    msg.header.frame_id = "board";
    msg.header.stamp = ros::Time::now();

    this->camera_pose_pub_.publish(msg);
  }

  void BoardEstimator::EstimateAndPublishPosition(const geometry_msgs::PoseStampedConstPtr& msg) {
    // get body center from camera center
    tf2::Vector3 v {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
    tf2::Transform tf (
      tf2::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w),
      tf2::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z)
    );
    geometry_msgs::PoseStamped body;
    body.header.stamp = msg->header.stamp;
    body.header.frame_id = msg->header.frame_id;
    tf2::toMsg(tf_camera_to_body_ * tf, body.pose);

    // then get body center w.r.t. map frame
    geometry_msgs::PoseStamped estimated_pose;
    tf2_buffer_.transform(body, estimated_pose, "map");
    
    this->estimated_pose_pub_.publish(estimated_pose);
  }

  void BoardEstimator::Init() {
    // listen for image
    source_sub_ =
      nh_.subscribe("source", 1,
			      &BoardEstimator::HandleImage, this);
    ROS_INFO("Listening for camera captured image source topic.");

    // listen for camera info
    camera_info_sub_ =
      nh_.subscribe("camera_info", 1, &BoardEstimator::HandleCameraInfo, this);

    // setting up static transforms
    // we will have a "board_center" frame, whose origin is at the
    // center of the board frame
    ROS_INFO("board width: %.2f, board height: %.2f. Creating static transform from board -> board_center at the center of the board.",
	     board_.width(), board_.height());

    std::vector<geometry_msgs::TransformStamped> static_tfs;

    auto tf_board_to_board_center = makeTransformStamped("board_center", "board",
							 -board_.center_x(), -board_.center_y(), 0,
							 0, 0, 0, 1);
    static_tfs.push_back(tf_board_to_board_center);

    std::string tf_map2boardcenter;
    nh_.getParam("tf_map2boardcenter", tf_map2boardcenter);
    geometry_msgs::TransformStamped tf_map_to_bc;
    tf_map_to_bc.header.frame_id = "map";
    tf_map_to_bc.header.stamp = ros::Time::now();
    tf_map_to_bc.child_frame_id = "board_center";
    parseTransformStringInto(tf_map_to_bc.transform, tf_map2boardcenter);
    static_tfs.push_back(tf_map_to_bc);

    static_transform_broadcaster_.sendTransform(static_tfs);

    std::string tf_camera2body;
    nh_.getParam("tf_camera2body", tf_camera2body);

    parseTransformStringInto(tf_camera_to_body_, tf_camera2body);
  }

}

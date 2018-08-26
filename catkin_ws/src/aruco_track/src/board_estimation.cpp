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

#include <Eigen/Geometry>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <opencv2/calib3d.hpp>
#include <memory>

#include "board_estimation.h"

#include <iostream>

using namespace std;
using namespace cv;
using namespace cv::aruco;

namespace aruco_track {

  bool ProcessFrame(const cv::InputArray& frame,
		    const Settings& settings,
		    cv::Mat& rvec, cv::Mat& tvec,
		    cv::OutputArray& undistorted) {
    cv::Mat helper;
    
    // processing step:
    // 1. undistort frame image
    if (undistorted.needed()) {
      undistort(frame, undistorted, settings.camera_matrix(), settings.distort_coeffs());
    } else {
      undistort(frame, helper, settings.camera_matrix(), settings.distort_coeffs());      
    }
  
    // 2. detect aruco markers
    vector<int> ids;
    vector<vector<Point2f> > corners;
    if (undistorted.needed()) {
      detectMarkers(undistorted, settings.markers_dict(), corners, ids);
    } else {
      detectMarkers(helper, settings.markers_dict(), corners, ids);
    }

    if (ids.size() == 0) {
      // no markers detected. skip frame.
      return false;
    }
  
    // 3. estimate pose
    int num_used = estimatePoseBoard(corners, ids, settings.board(),
				     settings.camera_matrix(),
				     settings.distort_coeffs(),
				     rvec,
				     tvec);

    if (num_used == 0) {
      // pose not estimated
      return false;
    }

    return true;
  }

  void DrawAxisOnImage(cv::InputOutputArray& image,
		       const Settings& settings,
		       const::Mat& rvec, const cv::Mat& tvec) {
    cv::aruco::drawAxis(image,
			settings.camera_matrix(),
			settings.distort_coeffs(),
			rvec, tvec,
			settings.board_marker_length());
  }

  /**
   * Broadcast dynamic transform between the world coordindate and the 
   * camera coordinate.
   */
  void BroadcastCameraTransform(const cv::Mat& rvec, const cv::Mat& tvec) {
    static tf2_ros::TransformBroadcaster br;
  
    static cv::Mat rot_mat;

    cv::Rodrigues(rvec, rot_mat);
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > mat(rot_mat.ptr<double>());

    Eigen::Quaternion<double> q(mat);
  
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = ros::Time::now();
    transform_stamped.header.frame_id = "world";
    transform_stamped.child_frame_id = "camera";

    transform_stamped.transform.translation.x = tvec.at<double>(0, 0);
    transform_stamped.transform.translation.y = tvec.at<double>(1, 0);
    transform_stamped.transform.translation.z = tvec.at<double>(2, 0);
    transform_stamped.transform.rotation.x = q.x();
    transform_stamped.transform.rotation.y = q.y();
    transform_stamped.transform.rotation.z = q.z();
    transform_stamped.transform.rotation.w = q.w();

    br.sendTransform(transform_stamped);
  }

}

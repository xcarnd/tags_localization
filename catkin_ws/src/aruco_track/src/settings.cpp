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

#include "settings.h"

namespace aruco_track {

  void Settings::UpdateArucoParameters(int markers_dict,
				       int board_num_x, int board_num_y,
				       double board_marker_length, double board_marker_separation) {
    board_num_x_ = board_num_x_;
    board_num_y_ = board_num_y_;
    board_marker_length_ = board_marker_length;
    board_marker_separation_ = board_marker_separation_;
    
    markers_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(markers_dict));
    board_ = cv::aruco::GridBoard::create(board_num_x,
					  board_num_y,
					  board_marker_length,
					  board_marker_separation,
    					  markers_dict_);
  }

  void Settings::UpdateWithCameraInfoMsg(const sensor_msgs::CameraInfoConstPtr& msg) {
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

    camera_info_updated_ = true;
  }
  
}


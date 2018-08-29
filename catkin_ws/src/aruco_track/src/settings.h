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

#ifndef _ARUCO_TRACK_SETTINGS_H_
#define _ARUCO_TRACK_SETTINGS_H_

#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <string>

namespace aruco_track {

  class Settings {
  private:
    bool camera_info_updated_;
    cv::Mat camera_matrix_;
    cv::Mat distort_coeffs_;
    cv::Ptr<cv::aruco::Dictionary> markers_dict_;
    cv::Ptr<cv::aruco::Board> board_;

    int board_num_x_;
    int board_num_y_;
    double board_marker_length_;
    double board_marker_separation_;

    bool publish_debug_image_;
    
  public:

    inline Settings() : publish_debug_image_(false) { }
    
    void UpdateArucoParameters(int markers_dict, int board_num_x, int board_num_y, double board_marker_length, double board_marker_separation);

    void UpdateWithCameraInfoMsg(const sensor_msgs::CameraInfoConstPtr& msg);
    
    inline const cv::Ptr<cv::aruco::Dictionary>& markers_dict() const { return markers_dict_; }
    inline const cv::Ptr<cv::aruco::Board>& board() const { return board_; }
    inline const cv::Mat& camera_matrix() const { return camera_matrix_; }
    inline const cv::Mat& distort_coeffs() const { return distort_coeffs_; }
    inline bool camera_info_updated() const { return camera_info_updated_; }
    inline bool publish_debug_image() const { return publish_debug_image_; }
    inline void set_publish_debug_image(bool publish) { publish_debug_image_ = publish; }
    inline int board_num_x() const { return board_num_x_; }
    inline int board_num_y() const { return board_num_y_; }
    inline double board_marker_length() const { return board_marker_length_; }
    inline double board_marker_separation() const { return board_marker_separation_; }
  };
  
}

#endif

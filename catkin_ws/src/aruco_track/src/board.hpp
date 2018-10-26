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

#ifndef _ARUCO_TRACK_BOARD_HPP_
#define _ARUCO_TRACK_BOARD_HPP_

#include <string>
#include <cstring>
#include <exception>

#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>

#include <ros/ros.h>

namespace aruco_track {

  class BoardParametersNotSetException : public std::exception {
    private:
      std::string field_name_;
    public:
      inline BoardParametersNotSetException(const std::string& field_name)
       : field_name_(field_name) {
      }
      virtual const char* what() const throw() {
          std::string msg = "Board field [" + field_name_ + "] not set properly";
          return msg.c_str();
      }
  };
  
  class Board {
  private:
    cv::Ptr<cv::aruco::Dictionary> markers_dict_;
    cv::Ptr<cv::aruco::Board> board_;

    int num_markers_x_;
    int num_markers_y_;
    double marker_size_;
    double marker_separation_;

    double width_;
    double height_;
    double center_x_;
    double center_y_;

  public:
    explicit inline Board(ros::NodeHandle& nh) {
      int dict;
      if (!nh.getParam("markers_dict", dict)) {
        throw BoardParametersNotSetException("marker_dict");
      }
      if (!nh.getParam("num_markers_x", num_markers_x_)) {
        throw BoardParametersNotSetException("num_markers_x");  
      }
      if (!nh.getParam("num_markers_y", num_markers_y_)) {
        throw BoardParametersNotSetException("num_markers_y");
      }
      if (!nh.getParam("marker_size", marker_size_)) {
        throw BoardParametersNotSetException("marker_size");  
      }
      if (!nh.getParam("marker_separation", marker_separation_)) {
        throw BoardParametersNotSetException("marker_separation");  
      }

      markers_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dict));
      board_ = cv::aruco::GridBoard::create(num_markers_x_,
                                            num_markers_y_,
					    marker_size_,
					    marker_separation_,
                      	                    markers_dict_);
      
      width_ = num_markers_x_ * marker_size_ + (num_markers_x_ - 1) * marker_separation_;
      height_ = num_markers_y_ * marker_size_ + (num_markers_y_ - 1) * marker_separation_;
      center_x_ = width_ / 2;
      center_y_ = height_ / 2;
    }

    inline const cv::Ptr<cv::aruco::Dictionary>& markers_dict() const { return markers_dict_; }
    inline const cv::Ptr<cv::aruco::Board>& board() const { return board_; }
    inline int num_markers_x() const { return num_markers_x_; }
    inline int num_markers_y() const { return num_markers_y_; }
    inline double marker_size() const { return marker_size_; }
    inline double marker_separation() const { return marker_separation_; }
    inline double width() const { return width_; }
    inline double height() const { return height_; }
    inline double center_x() const { return center_x_; }
    inline double center_y() const { return center_y_; }
  };
  
}

#endif

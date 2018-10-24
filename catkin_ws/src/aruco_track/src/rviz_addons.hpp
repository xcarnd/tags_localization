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

#ifndef _ARUCO_TRACK_RVIZ_ADDONS_HPP_
#define _ARUCO_TRACK_RVIZ_ADDONS_HPP_

#include "base_node.hpp"

namespace aruco_track {

  class RVizAddonsNode : public BaseNode {
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle parent_nh_;
  public:
    inline RVizAddonsNode(int argc, char **argv, const std::string& node_name)
      : BaseNode(argc, argv, node_name),
	nh_("~") { }

    inline int Run() {
      while (nh_.ok()) {
	ros::spin();
      }
      return 0;
    }
  };
  
}


#endif

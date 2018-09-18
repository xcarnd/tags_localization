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

#include <stdexcept>

#include <ros/ros.h>
#include <tf2/convert.h>

#include "locator_node.hpp"
#include "frame_def.h"

namespace aruco_track {

    LocatorNode::LocatorNode(int argv, char **argv, const std::string& node_name) 
    : nh_(std::NodeHandle("~")),
      tf_listener_(tf2_ros::TransformListener(tf_buffer_)) {
        pose_board_sub_ = nh_.subscribe("/aruco_track/board_pose", 1, &LocatorNode::HandleEstimatedPose, this);
        pose_map_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose", 1);
    }

    void LocatorNode::HandleEstimatedPose(const geometry_msgs::PoseStampedConstStr& msg) {
        geometry_msgs::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_.lookupTransform(FRAME_MAP, FRAME_BOARD, ros::Time(0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        // convert to fcu-equivalent coordinate in map
        geometry_msgs::PoseStamped fcu_eq_in_map;
        tf2::doTransform(*msg, fcu_eq_in_map, transform_stamped);
        pose_map_pub_.publish(fcu_eq_in_map);
    }

    int LocatorNode::Run() {
        try {
            while (nh_.ok()) {
                ros::spin();
            }
            return 0;
        } catch (...) {
            std::exception_ptr eptr = std::current_exception();
            ROS_ERROR("Exception caught: %s", eptr.what());
            return -1;
        }
    }

}

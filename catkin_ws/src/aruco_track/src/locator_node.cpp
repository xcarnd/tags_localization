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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "locator_node.hpp"
#include "frame_def.hpp"

namespace aruco_track {

    LocatorNode::LocatorNode(int argc, char **argv, const std::string& node_name) 
    : BaseNode(argc, argv, node_name),
      nh_(ros::NodeHandle("~")),
      tf_listener_(tf_buffer_) {
        ros::NodeHandle nh_aruco_track("aruco_track");
        pose_board_sub_ = nh_aruco_track.subscribe("board_pose", 1, &LocatorNode::HandleEstimatedPose, this);
        pose_map_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose", 1);
    }

    void LocatorNode::HandleEstimatedPose(const geometry_msgs::PoseStampedConstPtr& msg) {
      // how we need now is to report the estimated frame fcu_enu pose in frame map
      geometry_msgs::TransformStamped transform_stamped;
      try {
	  transform_stamped = tf_buffer_.lookupTransform(FRAME_MAP, FRAME_FCU_ENU, ros::Time(0));
      } catch (tf2::TransformException &ex) {
	ROS_WARN("%s", ex.what());
	return;
      }

      geometry_msgs::PoseStamped msg_pub;
      msg_pub.header.stamp = ros::Time::now();
      msg_pub.header.frame_id = FRAME_MAP;
      
      msg_pub.pose.position.x = transform_stamped.transform.translation.x;
      msg_pub.pose.position.y = transform_stamped.transform.translation.y;
      msg_pub.pose.position.z = transform_stamped.transform.translation.z;
      msg_pub.pose.orientation.x = transform_stamped.transform.rotation.x;
      msg_pub.pose.orientation.y = transform_stamped.transform.rotation.y;
      msg_pub.pose.orientation.z = transform_stamped.transform.rotation.z;
      msg_pub.pose.orientation.w = transform_stamped.transform.rotation.w;
      pose_map_pub_.publish(msg_pub);
    }

    int LocatorNode::Run() {
        try {
            while (nh_.ok()) {
                ros::spin();
            }
            return 0;
        } catch (std::exception& e) {
            ROS_ERROR("Exception caught: %s", e.what());
	    return -1;
        }
    }
}

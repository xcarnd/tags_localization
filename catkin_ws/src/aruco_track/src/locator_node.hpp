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

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

#include "base_node.hpp"

#ifndef _ARUCO_TRACK_LOCATOR_NODE_HPP_
#define _ARUCO_TRACK_LOCATOR_NODE_HPP_

namespace aruco_track {

    class LocatorNode : BaseNode {
    private:
        ros::NodeHandle nh_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        ros::Subscriber pose_board_sub_;

        ros::Publisher pose_map_pub_;
    private:
        void HandleEstimatedPose(const geometry_msgs::PoseStampedConstPtr& msg);
    public:
        LocatorNode(int argc, char **argv, const std::string& node_name = std::string("locator"));

        /**
         * Run the node until exit.
         */
        int Run();
    };

}

#endif


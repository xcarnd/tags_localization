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

#ifndef _ARUCO_TRACK_BASE_NODE_HPP_
#define _ARUCO_TRACK_BASE_NODE_HPP_

#include <string>
#include <exception>

#include <ros/ros.h>

namespace aruco_track {

    class NodeBase {
    protected:
        inline NodeBase(int argc, char **argv, const std::string& node_name) {
            ros::init(argc, argv, node_name);
        }
    };

    class RunnableNode : public NodeBase {
    protected:
        ros::NodeHandle parent_nh_;
        ros::NodeHandle nh_;
    protected:
        inline RunnableNode(int argc, char **argv, const std::string& node_name)
            : NodeBase(argc, argv, node_name),
              parent_nh_(""),
              nh_("~") { }
        virtual void Init() {}
    public:
        inline int Run() {
            try {
                this->Init();
                while (nh_.ok()) {
                    ros::spin();
                }
            } catch (const std::exception& e) {
                ROS_ERROR("Exception caught while running node.\n  what() => %s", e.what());
                return -1;
            }
        }
    };

}

#endif
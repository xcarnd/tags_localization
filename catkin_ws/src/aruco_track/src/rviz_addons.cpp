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

#include <cstring>

#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/utils.h>

#include "utils.hpp"
#include "rviz_addons.hpp"

namespace {
  inline geometry_msgs::Point makePoint(double x, double y, double z) {
    geometry_msgs::Point pt;
    pt.x = x;
    pt.y = y;
    pt.z = z;
    return pt;
  }

  inline std_msgs::ColorRGBA makeRGBA(double r, double g, double b, double a) {
    std_msgs::ColorRGBA rgba;
    rgba.r = r;
    rgba.g = g;
    rgba.b = b;
    rgba.a = a;
    return rgba;
  }
}

namespace aruco_track {

  void RVizAddonsNode::Init() {
    pose_sub_ = nh_.subscribe("pose", 1, &RVizAddonsNode::HandlePose, this);
    viz_object_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("object_viz", 1);
  }

  void RVizAddonsNode::HandlePose(const geometry_msgs::PoseStampedConstPtr& msg) {
    visualization_msgs::MarkerArray markerArray;
    // marker for the pose
    //
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::MODIFY;
    marker.pose.position = msg->pose.position;
    // the orientation part of msg is defined in the frame of fcu,
    // extra processing is needed to make the marker heading in the right direction.
    auto frame_q = makeTf2QuaternionFromRPYDegree(0, 0, 90);
    tf2::Quaternion pose_q = {
          msg->pose.orientation.x,
          msg->pose.orientation.y,
          msg->pose.orientation.z,
          msg->pose.orientation.w
    };
    auto marker_q = frame_q * pose_q;
    marker.pose.orientation.x = marker_q.getX();
    marker.pose.orientation.y = marker_q.getY();
    marker.pose.orientation.z = marker_q.getZ();
    marker.pose.orientation.w = marker_q.getW();
    
    marker.points = {
      makePoint(0.5, 0, 0), makePoint(-0.5, 0, 0), makePoint(0, 0.5 * 1.732, 0),
      makePoint(0, 0, 0), makePoint(0, 0.5 * 1.732, 0), makePoint(0, 0, 0.5)
    };
    marker.colors = {
      makeRGBA(1, 0, 0, 1), makeRGBA(1, 0, 0, 1), makeRGBA(0, 1, 0, 1),
      makeRGBA(1, 0, 0, 1), makeRGBA(0, 1, 0, 1), makeRGBA(0, 0, 1, 1),
    };
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    markerArray.markers.push_back(marker);

    // text for position and orientation (in euler angles)
    visualization_msgs::Marker text_marker;
    text_marker.header.frame_id = "map";
    text_marker.header.stamp = ros::Time();
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::MODIFY;
    text_marker.pose.position.x = 0;
    text_marker.pose.position.y = 0;
    text_marker.pose.position.z = -0.5;
    text_marker.pose.orientation.x = 0;
    text_marker.pose.orientation.y = 0;
    text_marker.pose.orientation.z = 0;
    text_marker.pose.orientation.w = 1;
    text_marker.scale.z = 0.1;
    text_marker.color.r = 1;
    text_marker.color.g = 1;
    text_marker.color.b = 1;
    text_marker.color.a = 1;

    tf2::Matrix3x3 m(pose_q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    char buf[1024];
    std::sprintf(buf, "Position: %.3f, %.3f, %.3f\nOrientation: %.1f, %.1f, %.1f",
              msg->pose.position.x,
              msg->pose.position.y,
              msg->pose.position.z,
              rad2deg(roll),
              rad2deg(pitch),
              rad2deg(yaw));
    text_marker.text = std::string(buf);
    markerArray.markers.push_back(text_marker);

    viz_object_pub_.publish(markerArray);
  }

}

using namespace aruco_track;

int main(int argc, char* argv[]) {
  
  RVizAddonsNode node(argc, argv, "rviz_addons");
  
  return node.Run();
  
}

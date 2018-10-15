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

#ifndef _ARUCO_TRACK_UTILS_HPP_
#define _ARUCO_TRACK_UTILS_HPP_

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/utils.h>

namespace {

  constexpr double PI = 3.14159265358979323846;

  inline void fillTransform(geometry_msgs::TransformStamped& transform,
			    const std::string &parent_frame,
			    const std::string &child_frame,
			    double tx, double ty, double tz,
			    double qx, double qy, double qz, double qw) {
    transform.header.frame_id = parent_frame;
    transform.child_frame_id = child_frame;
    
    transform.transform.translation.x = tx;
    transform.transform.translation.y = ty;
    transform.transform.translation.z = tz;

    transform.transform.rotation.x = qx;
    transform.transform.rotation.y = qy;
    transform.transform.rotation.z = qz;
    transform.transform.rotation.w = qw;
  }

  inline void fillTransform(geometry_msgs::TransformStamped& transform,
			    const std::string &parent_frame,
			    const std::string &child_frame,
			    const tf2::Vector3 &translation,
			    const tf2::Quaternion &rotation) {
    transform.header.frame_id = parent_frame;
    transform.child_frame_id = child_frame;
    
    transform.transform.translation.x = translation.getX();
    transform.transform.translation.y = translation.getY();
    transform.transform.translation.z = translation.getZ();

    transform.transform.rotation.x = rotation.getX();
    transform.transform.rotation.y = rotation.getY();
    transform.transform.rotation.z = rotation.getZ();
    transform.transform.rotation.w = rotation.getW();
  }
  
  inline void fillInverseIntoMsg(const geometry_msgs::Quaternion& quaternion, const geometry_msgs::Point& point, geometry_msgs::TransformStamped& msg) {
    tf2::Transform from(tf2::Quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w),
			tf2::Vector3(point.x, point.y, point.z));
    const tf2::Transform& inverse = from.inverse();
    msg.transform.translation.x = inverse.getOrigin().getX();
    msg.transform.translation.y = inverse.getOrigin().getY();
    msg.transform.translation.z = inverse.getOrigin().getZ();
    msg.transform.rotation.x = inverse.getRotation().getX();
    msg.transform.rotation.y = inverse.getRotation().getY();
    msg.transform.rotation.z = inverse.getRotation().getZ();
    msg.transform.rotation.w = inverse.getRotation().getW();
  }

  inline double deg2rad(double deg) {
    return deg * PI / 180;
  }
}

#endif


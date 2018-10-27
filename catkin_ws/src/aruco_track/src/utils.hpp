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

#include <string>
#include <sstream>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/utils.h>

#include <Eigen/Geometry>

namespace aruco_track {

  constexpr double PI = 3.14159265358979323846;

  inline double deg2rad(double deg) {
    return deg * PI / 180;
  }

  inline double rad2deg(double rad) {
    return rad * 180 / PI;
  }

  typedef enum {
    MEMBER_VARIABLE,
    MEMBER_METHOD,
    MEMBER_METHOD_GETTER,
    CUSTOM
  } _component_getter_type;

  template<typename T, _component_getter_type t>
  struct _component_getter {};

  template<typename T>
  struct _component_getter<T, MEMBER_VARIABLE> {
    inline static double x(const T& v) { return v.x; }
    inline static double y(const T& v) { return v.y; }
    inline static double z(const T& v) { return v.z; }
    inline static double w(const T& v) { return v.w; }
  };

  template<typename T>
  struct _component_getter<T, MEMBER_METHOD> {
    inline static double x(const T& v) { return v.x(); }
    inline static double y(const T& v) { return v.y(); }
    inline static double z(const T& v) { return v.z(); }
    inline static double w(const T& v) { return v.w(); }
  };

  template<typename T>
  struct _component_getter<T, MEMBER_METHOD_GETTER> {
    inline static double x(const T& v) { return v.getX(); }
    inline static double y(const T& v) { return v.getY(); }
    inline static double z(const T& v) { return v.getZ(); }
    inline static double w(const T& v) { return v.getW(); }
  };

  template<typename T>
  struct component_getter {};

  template<>
  struct component_getter<Eigen::Quaternion<double>>
    : _component_getter<Eigen::Quaternion<double>, MEMBER_METHOD> {};

  template<>
  struct component_getter<tf2::Quaternion>
    : _component_getter<tf2::Quaternion, MEMBER_METHOD_GETTER> {};
  
  template<>
  struct component_getter<geometry_msgs::Quaternion>
    : _component_getter<geometry_msgs::Quaternion, MEMBER_VARIABLE> {};
  
  template<>
  struct component_getter<geometry_msgs::Point>
    : _component_getter<geometry_msgs::Point, MEMBER_VARIABLE> {};
  

  inline
  tf2::Quaternion makeTf2QuaternionFromRPYDegree(double roll, double pitch, double yaw) {
    tf2::Quaternion q;
    q.setRPY(deg2rad(roll), deg2rad(pitch), deg2rad(yaw));
    return q;
  }

  template <typename T>
  void fillTransform(T& out, double tx, double ty, double tz, double qx, double qy, double qz, double qw);

  template <typename T, typename Q>
  inline void
  fillTransform(T& out,
                double tx, double ty, double tz,
                const Q& q) {
    fillTransform(out,
                  tx, ty, tz,
                  component_getter<Q>::x(q),
                  component_getter<Q>::y(q),
                  component_getter<Q>::z(q),
                  component_getter<Q>::w(q));
  }

  template <typename T, typename V>
  inline void
  fillTransform(T& out,
                const V& v,
                double qx, double qy, double qz, double qw) {
    fillTransform(out,
                  component_getter<V>::x(v),
                  component_getter<V>::y(v),
                  component_getter<V>::z(v),
                  qx, qy, qz, qw);
  }

  template <typename T, typename V, typename Q>
  inline void
  fillTransform(T& out, const V& v, const Q& q) {
    fillTransform(out,
                  component_getter<V>::x(v),
                  component_getter<V>::y(v),
                  component_getter<V>::z(v),
                  component_getter<Q>::x(q),
                  component_getter<Q>::y(q),
                  component_getter<Q>::z(q),
                  component_getter<Q>::w(q));
  }

  inline
  geometry_msgs::TransformStamped
  makeTransformStamped(const std::string &parent_frame,
		       const std::string &child_frame,
		       double tx, double ty, double tz,
		       double qx, double qy, double qz, double qw,
		       ros::Time timestamp = ros::Time::now()) {
    geometry_msgs::TransformStamped msg;
    msg.header.stamp = timestamp;
    msg.header.frame_id = parent_frame;
    msg.child_frame_id = child_frame;
    
    msg.transform.translation.x = tx;
    msg.transform.translation.y = ty;
    msg.transform.translation.z = tz;

    msg.transform.rotation.x = qx;
    msg.transform.rotation.y = qy;
    msg.transform.rotation.z = qz;
    msg.transform.rotation.w = qw;

    return msg;
  }

  template <typename Q>
  inline geometry_msgs::TransformStamped
  makeTransformStamped(const std::string &parent_frame,
		       const std::string &child_frame,
		       double tx, double ty, double tz,
		       const Q& q,
		       ros::Time timestamp = ros::Time::now()) {
    return makeTransformStamped(parent_frame, child_frame,
				tx, ty, tz,
				component_getter<Q>::x(q),
				component_getter<Q>::y(q),
				component_getter<Q>::z(q),
				component_getter<Q>::w(q),
				timestamp);
  }

  template <typename V>
  inline geometry_msgs::TransformStamped
  makeTransformStamped(const std::string &parent_frame,
		       const std::string &child_frame,
		       const V& v,
		       double qx, double qy, double qz, double qw,
		       ros::Time timestamp = ros::Time::now()) {
    return makeTransformStamped(parent_frame, child_frame,
				component_getter<V>::x(v),
				component_getter<V>::y(v),
				component_getter<V>::z(v),
				qx, qy, qz, qw,
				timestamp);
  }

  template <typename V, typename Q>
  inline geometry_msgs::TransformStamped
  makeTransformStamped(const std::string &parent_frame,
		       const std::string &child_frame,
		       const V& v,
		       const Q& q,
		       ros::Time timestamp = ros::Time::now()) {
    return makeTransformStamped(parent_frame, child_frame,
				component_getter<V>::x(v),
				component_getter<V>::y(v),
				component_getter<V>::z(v),
				component_getter<Q>::x(q),
				component_getter<Q>::y(q),
				component_getter<Q>::z(q),
				component_getter<Q>::w(q),
				timestamp);
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

  template <>
  inline void
  fillTransform<geometry_msgs::Transform>(geometry_msgs::Transform& out,
                double tx, double ty, double tz,
                double qx, double qy, double qz, double qw) {
    out.translation.x = tx;
    out.translation.y = ty;
    out.translation.z = tz;
    out.rotation.x = qx;
    out.rotation.y = qy;
    out.rotation.z = qz;
    out.rotation.w = qw;
  }

  template <>
  inline void
  fillTransform<tf2::Transform>(tf2::Transform& out,
                double tx, double ty, double tz,
                double qx, double qy, double qz, double qw) {
    ROS_ERROR ("<<<%.2f %.2f %.2f %.2f %.2f %.2f %.2f", tx, ty, tz, qx, qy, qz, qw);
    out.setOrigin(tf2::Vector3(tx, ty, tz));
    out.setRotation(tf2::Quaternion(qx, qy, qz, qw));
  }

  template <typename T>
  T parseTransformString(const std::string& str) {
    std::istringstream iss(str);
    double tx, ty, tz, r, p, y;
    iss>>tx>>ty>>tz>>r>>p>>y;

    T ret;
    fillTransform(
      ret,
      tx, ty, tz,
      makeTf2QuaternionFromRPYDegree(r, p, y));
    return ret;
  }

  template <typename T>
  void parseTransformStringInto(T& out, const std::string& str) {
    std::istringstream iss(str);
    double tx, ty, tz, r, p, y;
    iss>>tx>>ty>>tz>>r>>p>>y;
    fillTransform(out,
      tx, ty, tz,
      makeTf2QuaternionFromRPYDegree(r, p, y));
  }
}

#endif


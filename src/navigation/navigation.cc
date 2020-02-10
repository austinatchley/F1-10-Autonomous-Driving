//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "navigation.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "f1tenth_course/AckermannCurvatureDriveMsg.h"
#include "f1tenth_course/Pose2Df.h"
#include "f1tenth_course/VisualizationMsg.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/ros/ros_helpers.h"
#include "shared/util/timer.h"
#include "visualization/visualization.h"
#include <algorithm>

using Eigen::Rotation2D;
using Eigen::Vector2f;
using f1tenth_course::AckermannCurvatureDriveMsg;
using f1tenth_course::VisualizationMsg;
using std::max;
using std::min;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} // namespace

namespace navigation {

Navigation::Navigation(const string& map_file, const string& odom_topic, ros::NodeHandle& n,
                       float target_position, float target_curvature)
    : _n(n), _odom_topic(odom_topic), _startTime(_now()), _timeOfLastNav(0.f),
      _target_position(target_position), _target_curvature(target_curvature), _world_loc(0, 0),
      _world_angle(0), _world_vel(0, 0), _world_omega(0), _odom_loc(0, 0), _odom_loc_start(0, 0),
      _nav_complete(true), _nav_goal_loc(0, 0), _nav_goal_angle(0) {
  drive_pub_ = n.advertise<AckermannCurvatureDriveMsg>("ackermann_curvature_drive", 1);
  viz_pub_ = n.advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage("base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage("map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
  std::cout << "set nav goal" << std::endl;
  _timeOfLastNav = _now();

  _nav_complete = false;

  // nav_goal_loc_ = loc;
  // nav_goal_angle_ = angle;
}

void Navigation::ResetOdomFrame() {
  ros::topic::waitForMessage<nav_msgs::Odometry>(_odom_topic, _n);
  ros::spinOnce();

  _odom_loc_start = _odom_loc;
  _prev_odom_loc = _odom_loc;
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  // robot_loc_ = loc;
  // robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc, float angle, const Vector2f& vel,
                                float ang_vel) {
  _odom_loc = loc;
  _odom_angle = angle;
  _odom_vel = vel;
  _odom_omega = ang_vel;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud, double time) {}

void Navigation::_time_integrate() {
  const float timestep_duration = 1.0 / 20.0;

  // TODO: better integrator
  // const float d_theta = _odom_angle - _prev_odom_angle;
  // const Vector2f rel_d_x = _get_relative_coord(_odom_loc, _prev_odom_loc, d_theta);
  const Vector2f d_x = _odom_loc - _prev_odom_loc;
  _velocity = d_x / timestep_duration;
  _distance += d_x.norm();

  _prev_odom_angle = _odom_angle;
  _prev_odom_loc = _odom_loc;
}

void Navigation::Run() {
  const float timestep_duration = 1.0 / 20.0;
  const float system_latency = 0.1f;
  // const float time_sensor_data = now - system_latency;

  // TODO: Is this correct? Or do we know more about when this happens?
  // const float time_actuation = now + 2 * system_latency;

  _time_integrate();

  Vector2f direction(1, 0); // 1 for forward, -1 for backwards

  // approximate values of 1D position and speed at NEXT TIME STEP
  const float speed = _velocity.norm();
  const float position = _distance + speed * timestep_duration;
  
  const float predicted_speed = speed + (_last_accel * system_latency);
  const float predicted_position = position + (predicted_speed * system_latency);

  const float time_to_stop = (predicted_speed / MAX_DECEL) + system_latency;

  const float stop_position =
      predicted_position + (speed * time_to_stop) + (0.5 * MAX_DECEL * pow(time_to_stop, 2));

  float accel = 0.f;
  float calc_speed = 0.f;
  if (stop_position > _target_position) {
    // decelerate
    accel = -pow(predicted_speed, 2) / (2 * max(0.f, _target_position - predicted_position));
    calc_speed = predicted_speed;
  } else {
    accel = MAX_ACCEL;
    calc_speed = _toc_speed;
  }
  _toc_speed = min(MAX_SPEED, max(0.f, calc_speed + accel * timestep_duration));

  // Normalize the direction so we don't get a velocity greater than max
  direction = direction / direction.norm();

  AckermannCurvatureDriveMsg msg;
  msg.velocity = _toc_speed * direction[0];
  msg.curvature = _target_curvature;

  // msg.curvature = 1.f; // 1m radius of turning

  std::cout << "speed=" << speed << std::endl; 
  std::cout << "position=" << position << std::endl; 
  std::cout << "_odom_loc:" << std::endl << _odom_loc << std::endl;
  std::cout << "_target_position=" << _target_position << std::endl; 
  std::cout << "stop_position=" << stop_position << std::endl; 
  std::cout << "Sending velocity: " << msg.velocity << std::endl;
  std::cout << std::endl;

  drive_pub_.publish(msg);

  _last_accel = accel;
}

float Navigation::_now() {
  return ros::Time::now().toSec();
}

Vector2f Navigation::_get_relative_coord(Vector2f v1, Vector2f v2, float theta) {
  const Rotation2D<float> rotation(-theta);
  return rotation * (v2 - v1);
}

} // namespace navigation

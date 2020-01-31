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

Navigation::Navigation(const string& map_file, const string& odom_topic, ros::NodeHandle& n)
    : _n(n), _odom_topic(odom_topic), _startTime(now()), _timeOfLastNav(0.f), _navTime(0.f), _rampUpTime(0.f), _timeAtFullVel(0.f),
      _toc_speed(0), _world_loc(0, 0), _world_angle(0), _world_vel(0, 0), _world_omega(0), _odom_loc(0,0), _odom_loc_start(0,0),
      _nav_complete(true), _nav_goal_loc(0, 0), _nav_goal_angle(0) {
  drive_pub_ = n.advertise<AckermannCurvatureDriveMsg>("ackermann_curvature_drive", 1);
  viz_pub_ = n.advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage("base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage("map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
  std::cout << "set nav goal" << std::endl;
  _timeOfLastNav = now();

  _rampUpTime = (MAX_SPEED - _world_vel[0]) / MAX_ACCEL;
  _navTime = 10.f; // TODO: find total nav time
  _timeAtFullVel = _navTime - 2.f * _rampUpTime;

  _nav_complete = false;

  // nav_goal_loc_ = loc;
  // nav_goal_angle_ = angle;
}

void Navigation::ResetOdomFrame() {
  ros::topic::waitForMessage<nav_msgs::Odometry>(_odom_topic, _n);
  ros::spinOnce();

  _odom_loc_start = _odom_loc;
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

void Navigation::Run() {
  const float timeSinceLastNav = now() - _timeOfLastNav;

  const float timestep_duration = 1.0 / 20.0;
  // TODO: set based on nav target (forwards or backwards)
  const int direction = 1;
  const float target_position = 0.5;

  // TODO: speed at next timestep
  const float speed = _odom_vel.norm();
  // position at next timestep
  const float position = (_odom_loc - _odom_loc_start).norm() + speed * timestep_duration;
  std::cout << (_odom_loc - _odom_loc_start).norm() << std::endl;

  const float time_to_stop = speed / MAX_DECEL;
  const float stop_position =
      position + (speed * time_to_stop) + (0.5 * MAX_DECEL * pow(time_to_stop, 2));
  if (stop_position > target_position) {
    // decelerate
    const float decel = -pow(speed, 2) / (2 * max(0.f, target_position - position));
    _toc_speed = max(0.f, _toc_speed + decel * timestep_duration);
  } else {
    // accelerate
    _toc_speed = min(MAX_SPEED, _toc_speed + MAX_ACCEL * timestep_duration);
  }
  _toc_position += _toc_speed * timestep_duration;

  AckermannCurvatureDriveMsg msg;
  msg.velocity = _toc_speed * direction;

  // msg.curvature = 1.f; // 1m radius of turning

  std::cout << "Time elapsed since last nav command: " << timeSinceLastNav << std::endl;
  std::cout << "Sending velocity: " << msg.velocity << std::endl;

  drive_pub_.publish(msg);

  // Milestone 3 will complete the rest of navigation.
}

float Navigation::lerp(float a, float b, float t) {
  if (t > 1.f) {
    t = 1.f;
  }
  return a + t * (b - a);
}

float Navigation::now() {
  return ros::Time::now().toSec();
}

} // namespace navigation

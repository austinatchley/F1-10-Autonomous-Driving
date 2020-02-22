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
} // namespace

namespace navigation {
Navigation::Navigation(const string& map_file, const string& odom_topic, ros::NodeHandle& n,
                       float target_position, float target_curvature)
    : _n(n), _odom_topic(odom_topic), _target_position(target_position),
      _target_curvature(target_curvature), _world_loc(0, 0), _world_angle(0), _world_vel(0, 0),
      _world_omega(0), _odom_loc(0, 0), _odom_loc_start(0, 0), _nav_complete(true),
      _nav_goal_loc(0, 0), _nav_goal_angle(0) {
  drive_pub_ = n.advertise<AckermannCurvatureDriveMsg>("ackermann_curvature_drive", 1);
  viz_pub_ = n.advertise<VisualizationMsg>("visualization", 1);

  local_viz_msg_ = visualization::NewVisualizationMessage("base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage("map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
  std::cout << "set nav goal" << std::endl;

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

void Navigation::ObservePointCloud(double time) {
  for (auto point : point_cloud) {
    visualization::DrawCross(point, 0.1f, 0xd67d00, local_viz_msg_);
  }
}

void Navigation::_time_integrate() {
  // TODO: better integrator
  // const float d_theta = _odom_angle - _prev_odom_angle;
  // const Vector2f rel_d_x = _get_relative_coord(_odom_loc, _prev_odom_loc, d_theta);
  const Vector2f d_x = _odom_loc - _prev_odom_loc;
  _velocity = d_x / TIMESTEP;
  _distance += d_x.norm();

  _prev_odom_angle = _odom_angle;
  _prev_odom_loc = _odom_loc;
}

void Navigation::_update_vel_and_accel(float stop_position, float actuation_position,
                                       float actuation_speed, float target_position) {
  float output_accel = 0.f;
  float output_speed = 0.f;

  if (stop_position > target_position) {
    // decelerate
    const float remaining_distance = target_position - actuation_position;
    if (remaining_distance > 0) {
      output_accel = -pow(actuation_speed, 2) / (2 * remaining_distance);
      output_speed = actuation_speed;
    } else {
      output_accel = 0.f;
    }
  } else {
    output_accel = MAX_ACCEL;
    output_speed = _toc_speed;
  }

  // integrate desired acceleration
  _toc_speed = min(MAX_SPEED, max(0.f, output_speed + output_accel * TIMESTEP));
  _last_accel = output_accel;
}

AckermannCurvatureDriveMsg Navigation::_perform_toc(float distance, float curvature) {
  // past state at sensor read
  const float sensor_speed = _velocity.norm();
  const float sensor_position = _distance;

  // predicted current state accounting for sense -> run latency
  const float current_speed = sensor_speed + (_last_accel * LATENCY);
  const float current_position =
      sensor_position + (sensor_speed * LATENCY) + 0.5 * (_last_accel * pow(LATENCY, 2));

  // predicted future state at actuation of output from this control frame
  const float actuation_speed = current_speed; // assumes we have already reached previous output
                                               // velocity; we will not accelerate until actuation
  const float actuation_position =
      current_position + (current_speed * ACTUATION_LATENCY); // no acceleration

  const float time_to_stop = actuation_speed / MAX_DECEL;
  // predicted position at which car will come to rest
  const float stop_position =
      actuation_position + (actuation_speed * time_to_stop) + (MAX_DECEL * pow(time_to_stop, 2));

  _update_vel_and_accel(stop_position, actuation_position, actuation_speed, distance);

  AckermannCurvatureDriveMsg msg;
  msg.velocity = _toc_speed;
  msg.curvature = curvature;

  return msg;
}

bool Navigation::_is_in_straight_path(const Vector2f& p, float remaining_distance) {
  // std::cout << p[0] << ", " << p[1] << std::endl;
  return abs(p[1]) < CAR_W && p.x() > 0.f && p.x() < remaining_distance;
}

bool Navigation::_is_in_path(const Vector2f& p, float curvature, float remaining_distance, float r1,
                             float r2) {
  if (abs(curvature) < kEpsilon) {
    return _is_in_straight_path(p, remaining_distance);
  }

  Vector2f c(0, 1 / curvature);

  float r = (p - c).norm();

  float theta = std::atan2(p.x(), r - p[1]);

  return r >= r1 && r <= r2 && theta > 0;
}

float Navigation::_arc_distance(const Vector2f& p, float curvature) {
  if (abs(curvature) < kEpsilon) {
    return p.x();
  }

  const float r_turn = abs(1.f / curvature);

  float theta = std::atan2(p.x(), r_turn - abs(p.y()));
  return r_turn * theta;
}

float Navigation::_arc_distance_safe(const Vector2f& p, float curvature) {
  if (abs(curvature) < kEpsilon) {
    return p.x() - CAR_L;
  }

  const float r_turn = 1.f / curvature;

  float theta = std::atan2(p.x(), r_turn - p.y());
  float omega = std::atan2(CAR_L, r_turn - CAR_W);
  float phi = theta - omega;

  return r_turn * phi;
}

float Navigation::_get_free_path_length(float curvature) {
  float length = std::numeric_limits<float>::max();

  const float r_turn = 1.f / curvature;
  const float r1 = abs(r_turn) - CAR_W;
  const float r2 = sqrt(pow(abs(r_turn) + CAR_W, 2) + pow(CAR_L, 2));

  for (const Vector2f& point : point_cloud) {
    if (_is_in_path(point, curvature, length, r1, r2)) {
      const float point_dist = _arc_distance_safe(point, curvature);
      visualization::DrawCross(point, 0.1, 0x117dff, local_viz_msg_);
      if (point_dist < length) {
        length = point_dist;
      }
    }
  }

  return length;
}

Vector2f Navigation::_closest_approach(const float curvature, const Eigen::Vector2f& carrot) {
  if (abs(curvature) < kEpsilon) {
    return Vector2f(carrot.x(), 0);
  }

  const float radius = 1.f / curvature;
  const Vector2f center(0, radius);
  const Vector2f direction = (carrot - center).normalized();
  return center + direction * abs(radius);
}

float Navigation::_get_clearance(float curvature, float free_path_length) {
  float clearance = MAX_CLEARANCE;

  // Clamp to [-HALF_PI, HALF_PI]
  float theta_max = min(3.14f / 2.f, max(-3.14f / 2.f, curvature * free_path_length));

  if (abs(curvature) < kEpsilon) {
    for (const Vector2f& point : point_cloud) {
      if (point.x() < free_path_length && point.x() > 0.f) {
        clearance = min(clearance, max(0.f, abs(point.y()) - (CAR_W / 2.f)));
      }
    }

    return clearance;
  }

  const float r_turn = 1.f / curvature;
  const Vector2f center(0.f, r_turn);
  const float r1 = abs(r_turn) - CAR_W;
  const float r2 = sqrt(pow(abs(r_turn) + CAR_W, 2) + pow(CAR_L, 2));

  for (const Vector2f& point : point_cloud) {
    float theta = std::atan2(point.x(), r_turn - point.y());
    // std::cout << theta_max << std::endl;
    if (theta < 0 || theta > theta_max) {
      continue;
    }

    float point_radius = (center - point).norm();
    float cur_clearance = 0.f;
    if (point_radius < r1) {
      cur_clearance = r1 - point_radius;
    } else if (point_radius > r2) {
      cur_clearance = point_radius - r2;
    }

    clearance = min(cur_clearance, clearance);
  }

  return clearance;
}

float Navigation::_get_best_curvature() {
  const float min_curvature = -1;
  const float max_curvature = 1;
  const int n = 33;
  const float step_size = (max_curvature - min_curvature) / n;

  float max_score = 0;
  float best_curvature = 0;

  for (float curvature = min_curvature; curvature < max_curvature; curvature += step_size) {
    const Vector2f closest_approach = _closest_approach(curvature, _nav_goal_loc);
    visualization::DrawPoint(closest_approach, 0x107010, local_viz_msg_);

    const float free_path_length =
        min(_get_free_path_length(curvature), _arc_distance(closest_approach, curvature));

    const float distance_to_target = (_nav_goal_loc - closest_approach).norm();
    const float clearance = _get_clearance(curvature, free_path_length);
    const float score =
        free_path_length + WEIGHT_CLEARANCE * clearance + WEIGHT_DISTANCE * distance_to_target;

    if (score > max_score) {
      best_curvature = curvature;
      max_score = score;
    }
    visualization::DrawPathOption(curvature, free_path_length, 0, local_viz_msg_);
  }

  return best_curvature;
}

void Navigation::Run() {
  _time_integrate();

  _nav_goal_loc = Vector2f(3.f, 0.f);

  const float curvature = _get_best_curvature();
  float free_path_length = _get_free_path_length(curvature);
  visualization::DrawPathOption(curvature, free_path_length,
                                _get_clearance(curvature, free_path_length) + CAR_W, local_viz_msg_);

  float target_position = min(_target_position, _distance + free_path_length);

  auto msg = _perform_toc(target_position, curvature);

  visualization::DrawLine(Vector2f(0, -CAR_W), Vector2f(0, CAR_W), 0xff0000, local_viz_msg_);
  visualization::DrawLine(Vector2f(0, -CAR_W), Vector2f(CAR_L, -CAR_W), 0xff0000, local_viz_msg_);
  visualization::DrawLine(Vector2f(CAR_L, -CAR_W), Vector2f(CAR_L, CAR_W), 0xff0000,
                          local_viz_msg_);
  visualization::DrawLine(Vector2f(0, CAR_W), Vector2f(CAR_L, CAR_W), 0xff0000, local_viz_msg_);

  drive_pub_.publish(msg);
  viz_pub_.publish(local_viz_msg_);
  visualization::ClearVisualizationMsg(local_viz_msg_);
}

float Navigation::_now() {
  return ros::Time::now().toSec();
}

Vector2f Navigation::_get_relative_coord(Vector2f v1, Vector2f v2, float theta) {
  const Rotation2D<float> rotation(-theta);
  return rotation * (v2 - v1);
}

} // namespace navigation

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
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"
#include "shared/ros/ros_helpers.h"
#include "shared/util/timer.h"
#include "visualization/visualization.h"
#include <algorithm>
#include <chrono>

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
VisualizationMsg rtt_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
} // namespace

namespace navigation {
Navigation::Navigation(const string& map_file, const string& odom_topic, ros::NodeHandle& n,
                       float target_position, float target_curvature)
    : _n(n), _odom_topic(odom_topic), _target_position(target_position),
      _target_curvature(target_curvature), _world_loc(0, 0), _world_angle(0), _world_vel(0, 0),
      _world_omega(0), _odom_loc(0, 0), _odom_loc_start(0, 0), _carrot_loc(0, 0),
      _nav_complete(true), _nav_goal_loc(0, 0), _nav_goal_angle(0), _rrt(_map, global_viz_msg_) {
  drive_pub_ = n.advertise<AckermannCurvatureDriveMsg>("ackermann_curvature_drive", 1);
  viz_pub_ = n.advertise<VisualizationMsg>("visualization", 1);

  local_viz_msg_ = visualization::NewVisualizationMessage("base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage("map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);

  _map.Load(map_file);
  _rrt.Initialize();
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
  _nav_complete = false;
  _rrt.StartFindPath(_world_loc, loc);

  _nav_goal_loc = loc;
  _nav_goal_angle = angle;
}

void Navigation::ResetOdomFrame() {
  ros::topic::waitForMessage<nav_msgs::Odometry>(_odom_topic, _n);
  ros::spinOnce();

  _odom_loc_start = _odom_loc;
  _prev_odom_loc = _odom_loc;
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  _world_loc = loc;
  _world_angle = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc, float angle, const Vector2f& vel,
                                float ang_vel) {
  _odom_loc = loc;
  _odom_angle = angle;
  _odom_vel = vel;
  _odom_omega = ang_vel;
}

void Navigation::ObservePointCloud(double time) {}

void Navigation::_time_integrate() {
  // TODO: better integrator
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

  const float r_turn = abs(1.f / curvature);

  float theta = std::atan2(p.x(), r_turn - abs(p.y()));
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
      // visualization::DrawCross(point, 0.1, 0x117dff, local_viz_msg_);
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

void Navigation::_get_clearance(float& min_clearance, float& avg_clearance, float curvature,
                                float free_path_length) {
  if (free_path_length < 0.f) {
    min_clearance = 0.f;
    avg_clearance = 0;
    return;
  }

  min_clearance = MAX_CLEARANCE;
  avg_clearance = 0;

  int points = 0;
  if (abs(curvature) < kEpsilon) {
    for (const Vector2f& point : point_cloud) {
      if (point.x() < free_path_length && point.x() > 0.f) {
        const float clearance = max(0.f, abs(point.y()) - CAR_W);
        min_clearance = min(min_clearance, clearance);
        avg_clearance += clearance;

        points++;
      }
    }

    avg_clearance = avg_clearance / points;
    return;
  }

  const float r_turn = 1.f / curvature;
  const Vector2f center(0.f, r_turn);
  const float r1 = abs(r_turn) - CAR_W;
  const float r2 = sqrt(pow(abs(r_turn) + CAR_W, 2) + pow(CAR_L, 2));

  for (const Vector2f& point : point_cloud) {
    const float arc_distance = _arc_distance(point, curvature);
    if (arc_distance < 0 || arc_distance > free_path_length) {
      continue;
    }

    float point_radius = (center - point).norm();
    float cur_clearance = 0.f;
    if (point_radius < r1) {
      cur_clearance = r1 - point_radius;
    } else if (point_radius > r2) {
      cur_clearance = point_radius - r2;
    }

    min_clearance = min(cur_clearance, min_clearance);
    avg_clearance += cur_clearance;
    points++;
  }

  avg_clearance = avg_clearance / points;
}

float Navigation::_path_score(float curvature) {
  const Vector2f closest_approach = _closest_approach(curvature, _carrot_loc);

  const float free_path_length = min(_get_free_path_length(curvature), 5.f);

  const float distance_to_target = (_carrot_loc - closest_approach).norm();

  float min_clearance, avg_clearance;
  _get_clearance(min_clearance, avg_clearance, curvature, free_path_length);

  const float wall_avoidance = -max(0.f, WALL_AVOID_DISTANCE - min_clearance) * WEIGHT_AVOID_WALLS;

  const float score =
      free_path_length + pow(free_path_length, 2.0) * WEIGHT_CLEARANCE * min_clearance +
      WEIGHT_AVG_CLEARANCE * avg_clearance + WEIGHT_DISTANCE * distance_to_target + wall_avoidance;

  visualization::DrawPathOption(curvature, free_path_length, score, local_viz_msg_);
  return score;
}

float Navigation::_get_best_curvature() {
  const float min_curvature = -1;
  const float max_curvature = 1;
  const int n = 51;
  const float step_size = (max_curvature - min_curvature) / (n - 1);

  float max_score = 0;
  float best_curvature = 0;

  // coarse search
  for (float curvature = min_curvature; curvature < max_curvature; curvature += step_size) {
    const float score = _path_score(curvature);
    if (score > max_score) {
      best_curvature = curvature;
      max_score = score;
    }
  }

  float c0 = max(best_curvature - step_size, min_curvature);
  float c1 = min(best_curvature + step_size, max_curvature);
  return _golden_section_search(c0, c1);
}

/**
 * Perform Golden Section Search to find local max of path score
 * on the interval we found using naive search
 */
float Navigation::_golden_section_search(float c0, float c1) {
  static constexpr float phi = 0.618;

  for (int i = 0; i < 12; ++i) {
    float c2 = c1 - (c1 - c0) * phi;
    float c3 = c0 + (c1 - c0) * phi;

    if (_path_score(c2) > _path_score(c3)) {
      c1 = c3;
    } else {
      c0 = c2;
    }
  }

  return (c0 + c1) / 2.f;
}

void Navigation::_update_global_path() {
  if (_nav_complete)
    return;
  
  if (_rrt.ReachedGoal(_world_loc, _nav_goal_loc)) {
    _nav_complete = true;
    _path.clear();
    return;
  }

  // restart path planning if necessary
  if (!_planned_path_valid()) {
    _rrt.StartFindPath(_world_loc, _nav_goal_loc);
  }

  // if path is not fully resolved, keep running RRT
  if (_rrt.IsFindingPath()) {
    // time path update
    // auto now = std::chrono::high_resolution_clock::now();
    size_t iterations = 0;
    _rrt.FindPath(_path, iterations);
    // auto dt = std::chrono::high_resolution_clock::now() - now;
    // auto us = std::chrono::duration_cast<std::chrono::microseconds>(dt).count();
    // std::cerr << us / iterations << std::endl;
  }
}

void Navigation::Run() {
  _time_integrate();
  _update_global_path();

  AckermannCurvatureDriveMsg msg; 
  if (!_nav_complete) {
    // compute local carrot from global path
    const Vector2f carrot_global = _find_carrot();
    _carrot_loc = Eigen::Rotation2D<float>(-_world_angle) * (carrot_global - _world_loc);
    visualization::DrawCross(carrot_global, 0.2f, 0xAF6900, global_viz_msg_);

    // perform local navigation
    const float curvature = _get_best_curvature();
    float free_path_length = _get_free_path_length(curvature);
    float target_position = min(_target_position, _distance + free_path_length);
    msg = _perform_toc(target_position, curvature);

    // visualize best path option
    float min_clearance, avg_clearance;
    _get_clearance(min_clearance, avg_clearance, curvature, free_path_length);
    visualization::DrawPathOption(curvature, free_path_length, min_clearance + CAR_W, local_viz_msg_);
  }
  drive_pub_.publish(msg);

  // visualize nav goal
  visualization::DrawCross(_nav_goal_loc, 0.3f, 0xAF6900, global_viz_msg_);

  // visualize global pathing
  _rrt.VisualizeCurrentTree();
  _rrt.VisualizePath(_path);

  // visualize robot outline
  visualization::DrawLine(Vector2f(0, -CAR_W), Vector2f(0, CAR_W), 0xff0000, local_viz_msg_);
  visualization::DrawLine(Vector2f(0, -CAR_W), Vector2f(CAR_L, -CAR_W), 0xff0000, local_viz_msg_);
  visualization::DrawLine(Vector2f(CAR_L, -CAR_W), Vector2f(CAR_L, CAR_W), 0xff0000,
                          local_viz_msg_);
  visualization::DrawLine(Vector2f(0, CAR_W), Vector2f(CAR_L, CAR_W), 0xff0000, local_viz_msg_);

  viz_pub_.publish(local_viz_msg_);
  visualization::ClearVisualizationMsg(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);
}

float Navigation::_now() {
  return ros::Time::now().toSec();
}

Vector2f Navigation::_find_carrot() {
  static constexpr float radius = 2.f;

  Vector2f carrot(0.f, 0.f);
  if (_path.empty()) {
    return carrot;
  }

  float sqdist = 0.f;
  Vector2f intersection;
  for (int i = _path.size() - 1; i > 0; --i) {
    if (geometry::FurthestFreePointCircle(_path.at(i).loc, _path.at(i - 1).loc, _world_loc, radius,
                                          &sqdist, &intersection)) {
      break;
    }
  }

  return intersection;
}

bool Navigation::_planned_path_valid() {
  static constexpr float max_dist_to_edge = 0.5;

  for (int i = _path.size() - 1; i >= 1; --i) {
    const Vector2f& v0 = _path.at(i).loc;
    const Vector2f& v1 = _path.at(i - 1).loc;

    const Vector2f& projected_point = geometry::ProjectPointOntoLineSegment(_world_loc, v0, v1);
    const float dist_to_edge = (_world_loc - projected_point).norm();

    if (dist_to_edge < max_dist_to_edge) {
      return true;
    }
  }

  return false;
}

Vector2f Navigation::_get_relative_coord(Vector2f v1, Vector2f v2, float theta) {
  const Rotation2D<float> rotation(-theta);
  return rotation * (v2 - v1);
}

} // namespace navigation

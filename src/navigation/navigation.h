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
\file    navigation.h
\brief   Interface for reference Navigation class.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <vector>

#include "eigen3/Eigen/Dense"
#include "f1tenth_course/AckermannCurvatureDriveMsg.h"

#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace ros {
class NodeHandle;
} // namespace ros

namespace navigation {
static constexpr float RATE = 20.f;
static constexpr float TIMESTEP = 1.f / RATE;

static constexpr float CAR_W = .281 / 2 + 0.05f; // Half the width of the car
static constexpr float CAR_L = .43 + 0.1;      // Length of car
static constexpr float LASER_OFFSET = .2;      // Distance between laser and base_link

struct PathOption {
  float curvature;
  float clearance;
  float free_path_length;
  Eigen::Vector2f obstruction;
  Eigen::Vector2f closest_point;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

class Navigation {
public:
  static constexpr float MAX_SPEED = 1.f;
  static constexpr float MAX_ACCEL = 3.f;
  static constexpr float MAX_DECEL = 3.f;

  static constexpr float LATENCY = 0.085f;
  static constexpr float ACTUATION_LATENCY = LATENCY;

  static constexpr float WEIGHT_CLEARANCE = 6.f;
  static constexpr float WEIGHT_AVG_CLEARANCE = 0.f;
  static constexpr float WEIGHT_DISTANCE = 0.0f;
  static constexpr float WEIGHT_AVOID_WALLS = 0.f;
  static constexpr float WALL_AVOID_DISTANCE = 0.1f;

  static constexpr float MAX_CLEARANCE = .33f;

  // Epsilon value for handling limited numerical precision.
  static constexpr float kEpsilon = 1e-5;

  // Constructor
  explicit Navigation(const std::string& map_file, const std::string& odom_topic,
                      ros::NodeHandle& n, float target_position, float target_curvature);

  // Used in callback from localization to update position.
  void UpdateLocation(const Eigen::Vector2f& loc, float angle);

  // Used in callback for odometry messages to update based on odometry.
  void UpdateOdometry(const Eigen::Vector2f& loc, float angle, const Eigen::Vector2f& vel,
                      float ang_vel);

  // Updates based on an observed laser scan
  void ObservePointCloud(double time);

  // Main function called continously from main
  void Run();
  // Used to set the next target pose.
  void SetNavGoal(const Eigen::Vector2f& loc, float angle);

  void ResetOdomFrame();

  // the point cloud. updated on each ObservePointCloud call
  std::vector<Eigen::Vector2f> point_cloud;

private:
  float _now();

  void _time_integrate();
  void _update_vel_and_accel(float stop_position, float actuation_position, float actuation_speed,
                             float target_position);

  f1tenth_course::AckermannCurvatureDriveMsg _perform_toc(float distance, float curvature);

  bool _is_in_path(const Eigen::Vector2f& point, float curvature, float remaining_distance,
                   float r1, float r2);
  bool _is_in_straight_path(const Eigen::Vector2f& point, float remaining_distance);

  float _arc_distance_safe(const Eigen::Vector2f& p, float curvature);
  float _arc_distance(const Eigen::Vector2f& p, float curvature);
  Eigen::Vector2f _closest_approach(const float curvature, const Eigen::Vector2f& carrot);

  float _path_score(float curvature);
  float _get_best_curvature();
  float _golden_section_search(float c0, float c1);
  float _get_free_path_length(float curvature);
  void _get_clearance(float& min_clearance, float& avg_clearance, float curvature, float free_path_length);

  Eigen::Vector2f _get_relative_coord(Eigen::Vector2f v1, Eigen::Vector2f v2, float theta);

  ros::NodeHandle& _n;
  const std::string& _odom_topic;

  float _target_position;
  float _target_curvature;

  // current TOC speed output
  float _toc_speed = 0.f;

  float _distance = 0.f;                                 // distance travelled in odom frame
  Eigen::Vector2f _velocity = Eigen::Vector2f(0.f, 0.f); // velocity in odom frame

  float _last_accel = 0.f;

  // Current robot location.
  Eigen::Vector2f _world_loc;
  // Current robot orientation.
  float _world_angle;
  // Current robot velocity.
  Eigen::Vector2f _world_vel;
  // Current robot angular speed.
  float _world_omega;

  // Odometry-reported robot location.
  Eigen::Vector2f _odom_loc;
  // Odometry-reported location from the last time step
  Eigen::Vector2f _prev_odom_loc = Eigen::Vector2f(0.f, 0.f);

  // Odometry-reported robot angle.
  float _odom_angle;
  // Odometry-reported angle from the last time step
  float _prev_odom_angle = 0.f;

  // Odometry-reported robot velocity.
  Eigen::Vector2f _odom_vel;

  // Odometry-reported robot angular speed.
  float _odom_omega;

  Eigen::Vector2f _odom_loc_start;

  // Whether navigation is complete.
  bool _nav_complete;
  // Navigation goal location.
  Eigen::Vector2f _nav_goal_loc;
  // Navigation goal angle.
  float _nav_goal_angle;
};

} // namespace navigation

#endif // NAVIGATION_H

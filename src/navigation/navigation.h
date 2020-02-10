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

#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace ros {
class NodeHandle;
} // namespace ros

namespace navigation {

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
  static constexpr float MAX_ACCEL = 1.f;
  static constexpr float MAX_DECEL = 3.f;

  // Constructor
  explicit Navigation(const std::string& map_file, const std::string& odom_topic,
                      ros::NodeHandle& n, float target_position, float target_curvature);

  // Used in callback from localization to update position.
  void UpdateLocation(const Eigen::Vector2f& loc, float angle);

  // Used in callback for odometry messages to update based on odometry.
  void UpdateOdometry(const Eigen::Vector2f& loc, float angle, const Eigen::Vector2f& vel,
                      float ang_vel);

  // Updates based on an observed laser scan
  void ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud, double time);

  // Main function called continously from main
  void Run();
  // Used to set the next target pose.
  void SetNavGoal(const Eigen::Vector2f& loc, float angle);

  void ResetOdomFrame();

private:
  float _now();

  void _time_integrate();

  Eigen::Vector2f _get_relative_coord(Eigen::Vector2f v1, Eigen::Vector2f v2, float theta);

  ros::NodeHandle& _n;
  const std::string& _odom_topic;

  float _startTime;
  float _timeOfLastNav;
  float _target_position;
  float _target_curvature;

  // current TOC speed output
  float _toc_speed = 0.f;

  float _distance = 0.f;                                 // distance travelled in odom frame
  Eigen::Vector2f _position = Eigen::Vector2f(0.f, 0.f); // position in odom frame
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

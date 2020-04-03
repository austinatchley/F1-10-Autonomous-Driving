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
\file    particle-filter-main.cc
\brief   Main entry point for particle filter based
         mobile robot localization
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <inttypes.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "f1tenth_course/VisualizationMsg.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "gflags/gflags.h"
#include "nav_msgs/Odometry.h"
#include "ros/package.h"
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/LaserScan.h"

#include "config_reader/config_reader.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "particle_filter.h"
#include "vector_map/vector_map.h"
#include "visualization/visualization.h"

using Eigen::Vector2f;
using f1tenth_course::VisualizationMsg;
using geometry::Line;
using geometry::line2f;
using math_util::DegToRad;
using math_util::RadToDeg;
using ros::Time;
using std::string;
using std::vector;
using visualization::ClearVisualizationMsg;
using visualization::DrawArc;
using visualization::DrawLine;
using visualization::DrawParticle;
using visualization::DrawPoint;

// Create command line arguements
DEFINE_string(laser_topic, "/scan", "Name of ROS topic for LIDAR data");
DEFINE_string(odom_topic, "/odom", "Name of ROS topic for odometry data");
DEFINE_string(init_topic, "/initialpose", "Name of ROS topic for initialization");
DEFINE_string(map, "", "Map file to use");

DECLARE_int32(v);

// Create config reader entries
CONFIG_STRING(map_name_, "map");
CONFIG_FLOAT(init_x_, "init_x");
CONFIG_FLOAT(init_y_, "init_y");
CONFIG_FLOAT(init_r_, "init_r");
config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

bool run_ = true;
particle_filter::ParticleFilter particle_filter_;
ros::Publisher visualization_publisher_;
ros::Publisher localization_publisher_;
ros::Publisher laser_publisher_;
VisualizationMsg vis_msg_;
sensor_msgs::LaserScan last_laser_msg_;
vector_map::VectorMap map_;
vector<Vector2f> trajectory_points_;
Vector2f reference_loc_;
double reference_angle_;

void InitializeMsgs() {
  std_msgs::Header header;
  header.frame_id = "map";
  header.seq = 0;

  vis_msg_ = visualization::NewVisualizationMessage("map", "particle_filter");
}

void PublishParticles() {
  vector<particle_filter::Particle> particles;
  particle_filter_.GetParticles(&particles);
  if (particles.size() == 0)
    return;
  
  double max_weight = 0;
  // visualize particles
  for (const particle_filter::Particle& p : particles) {
    DrawParticle(p.loc, p.angle, vis_msg_);
    max_weight = std::min(max_weight, p.weight);
  }

  // visualize particle weights
  for (const particle_filter::Particle& p : particles) {
    const auto col_map = [](const double f){return static_cast<int>(floor(std::max(0.0, std::min(1.0, f)) * 0xFF));};
    const uint color = (col_map(p.weight / max_weight) << 16) | (col_map(1 - p.weight / max_weight) << 8) | 0x77;
    DrawPoint(p.loc, color, vis_msg_);
  }

  // show best hypothesis
  Vector2f loc;
  float angle;
  particle_filter_.GetLocation(&loc, &angle);  
  DrawLine(loc, loc + Vector2f(cos(angle), sin(angle)), 0xFF0000, vis_msg_);
  DrawLine(reference_loc_, reference_loc_ + Vector2f(cos(reference_angle_), sin(reference_angle_)), 0x70FFFF, vis_msg_);
  
  for (uint i = 0; i < last_laser_msg_.ranges.size(); ++i) {
    const double laser_range = last_laser_msg_.ranges[i];
    const double laser_angle = last_laser_msg_.angle_min + i * last_laser_msg_.angle_increment;
    DrawPoint(loc + Vector2f(cos(angle + laser_angle) * laser_range, sin(angle + laser_angle) * laser_range), 0x70FFFF, vis_msg_);
  }
}

void PublishPredictedScan() {
  const uint32_t kColor = 0xd67d00;
  Vector2f robot_loc(0, 0);
  float robot_angle(0);
  particle_filter_.GetLocation(&robot_loc, &robot_angle);
  vector<Vector2f> predicted_scan;
  particle_filter_.GetPredictedPointCloud(robot_loc, robot_angle, last_laser_msg_.ranges.size(),
                                          last_laser_msg_.range_min, last_laser_msg_.range_max,
                                          last_laser_msg_.angle_min, last_laser_msg_.angle_max, last_laser_msg_.angle_increment,
                                          &predicted_scan);
  for (const Vector2f& p : predicted_scan) {
    DrawPoint(p, kColor, vis_msg_);
  }
}

void PublishTrajectory() {
  const uint32_t kColor = 0xadadad;
  Vector2f robot_loc(0, 0);
  float robot_angle(0);
  particle_filter_.GetLocation(&robot_loc, &robot_angle);
  static Vector2f last_loc_(0, 0);
  if (!trajectory_points_.empty() && (last_loc_ - robot_loc).squaredNorm() > Sq(1.5)) {
    trajectory_points_.clear();
  }
  if (trajectory_points_.empty() || (robot_loc - last_loc_).squaredNorm() > 0.25) {
    trajectory_points_.push_back(robot_loc);
    last_loc_ = robot_loc;
  }
  for (size_t i = 0; i + 1 < trajectory_points_.size(); ++i) {
    DrawLine(trajectory_points_[i], trajectory_points_[i + 1], kColor, vis_msg_);
  }
}

void PublishVisualization() {
  static double t_last = 0;
  if (GetMonotonicTime() - t_last < 0.05) {
    // Rate-limit visualization.
    return;
  }
  t_last = GetMonotonicTime();
  vis_msg_.header.stamp = ros::Time::now();
  ClearVisualizationMsg(vis_msg_);

  PublishParticles();
  PublishPredictedScan();
  PublishTrajectory();
  visualization_publisher_.publish(vis_msg_);
}

void LaserCallback(const sensor_msgs::LaserScan& msg) {
  if (FLAGS_v > 0) {
    printf("Laser t=%f\n", msg.header.stamp.toSec());
  }
  last_laser_msg_ = msg;
  particle_filter_.ObserveLaser(msg.ranges, msg.range_min, msg.range_max, msg.angle_min,
                                msg.angle_max);
}

void OdometryCallback(const nav_msgs::Odometry& msg) {
  if (FLAGS_v > 0) {
    printf("Odometry t=%f\n", msg.header.stamp.toSec());
  }
  const Vector2f odom_loc(msg.pose.pose.position.x, msg.pose.pose.position.y);
  const float odom_angle = 2.0 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  particle_filter_.ObserveOdometry(odom_loc, odom_angle);
  Vector2f robot_loc(0, 0);
  float robot_angle(0);
  particle_filter_.GetLocation(&robot_loc, &robot_angle);
  geometry_msgs::Pose2D localization_msg;
  localization_msg.x = robot_loc.x();
  localization_msg.y = robot_loc.y();
  localization_msg.theta = robot_angle;
  localization_publisher_.publish(localization_msg);
}

void InitCallback(const nav_msgs::Odometry& msg) {
  const Vector2f init_loc(msg.pose.pose.position.x, msg.pose.pose.position.y);
  const float init_angle = 2.0 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  const string map = FLAGS_map.empty() ? CONFIG_map_name_ : FLAGS_map;
  printf("Initialize: %s (%f,%f) %f\u00b0\n", map.c_str(), init_loc.x(), init_loc.y(),
         RadToDeg(init_angle));
  particle_filter_.Initialize(map, init_loc, init_angle);
  trajectory_points_.clear();
}

void InitPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg) {
  const float x = msg.pose.pose.position.x;
  const float y = msg.pose.pose.position.y;
  const float theta = 2.0 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  const string map = FLAGS_map.empty() ? CONFIG_map_name_ : FLAGS_map;
  printf("Initialize: %s (%f,%f) %f\u00b0\n", map.c_str(), x, y, RadToDeg(theta));
  particle_filter_.Initialize(map, Vector2f(x, y), theta);
}

void ReferenceCallback(const geometry_msgs::Pose2D& msg) {
  reference_loc_ = Vector2f(msg.x, msg.y);
  reference_angle_ = msg.theta;
}

void ProcessLive(ros::NodeHandle* n) {
  ros::Subscriber initial_pose_sub = n->subscribe(FLAGS_init_topic.c_str(), 1, InitPoseCallback);
  ros::Subscriber laser_sub = n->subscribe(FLAGS_laser_topic.c_str(), 1, LaserCallback);
  ros::Subscriber odom_sub = n->subscribe(FLAGS_odom_topic.c_str(), 1, OdometryCallback);
  ros::Subscriber reference_sub = n->subscribe("/reference_localization", 1, ReferenceCallback);
  
  while (ros::ok() && run_) {
    ros::spinOnce();
    PublishVisualization();
    Sleep(0.01);
  }
}

void SignalHandler(int) {
  if (!run_) {
    printf("Force Exit.\n");
    exit(0);
  }
  printf("Exiting.\n");
  run_ = false;
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  signal(SIGINT, SignalHandler);
  // Initialize ROS.
  ros::init(argc, argv, "particle_filter", ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  InitializeMsgs();
  map_ = vector_map::VectorMap(CONFIG_map_name_);

  visualization_publisher_ = n.advertise<VisualizationMsg>("visualization", 1);
  localization_publisher_ = n.advertise<geometry_msgs::Pose2D>("localization", 1);
  laser_publisher_ = n.advertise<sensor_msgs::LaserScan>("scan", 1);

  ProcessLive(&n);

  return 0;
}

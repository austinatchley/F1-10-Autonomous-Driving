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
\file    particle-filter.cc
\brief   Particle Filter Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include <algorithm>
#include <cmath>
#include <iostream>

#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"

using Eigen::Vector2f;
using Eigen::Vector2i;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using vector_map::VectorMap;

DEFINE_double(num_particles, 50, "Number of particles");

// Fill in the body of these functions and create helpers as needed
// in order to implement localization using a particle filter.

// Milestone 2 will be implemented here.

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter()
    : _prev_odom_loc(0, 0), _prev_odom_angle(0), _odom_initialized(false) {}

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = _particles;
}

void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc, const float angle, int num_ranges,
                                            float range_min, float range_max, float angle_min,
                                            float angle_max, vector<Vector2f>* scan_ptr) {}

void ParticleFilter::Update(const vector<float>& ranges, float range_min, float range_max,
                            float angle_min, float angle_max, Particle* p_ptr) {}

void ParticleFilter::Resample() {}

void ParticleFilter::ObserveLaser(const vector<float>& ranges, float range_min, float range_max,
                                  float angle_min, float angle_max) {}

void ParticleFilter::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
  // TODO: Adjust this value to fit the physical behavior of the car
  static constexpr double k1 = 0.1;
  static constexpr double k2 = 0.05;
  static constexpr double k3 = 0.01;
  static constexpr double k4 = 0.05;
  static constexpr double k5 = 0.05;
  static constexpr double k6 = 0.1; 

  if (not _odom_initialized) {
    _prev_odom_loc = odom_loc;
    _prev_odom_angle = odom_angle;

    _odom_initialized = true;
    return;
  }

  const Vector2f dx = odom_loc - _prev_odom_loc;
  const double len = dx.norm();

  const float da = math_util::AngleDiff(odom_angle, _prev_odom_angle);

  static std::vector<Particle*> resample_vector;
  resample_vector.reserve(_particles.size());
  resample_vector.clear();

  for (Particle& p : _particles) {
    const Vector2f prev_loc = p.loc;

    const Vector2f dir(cos(p.angle), sin(p.angle));
    const Vector2f perp_dir(cos(p.angle + M_PI / 2.f), sin(p.angle + M_PI / 2.f));

    const Vector2f e_x = dir * _rng.Gaussian(0, k1 * len + k2 * abs(da));
    const Vector2f e_y = perp_dir * _rng.Gaussian(0, k3 * len + k4 * abs(da));

    p.loc += (dir * len) + e_x + e_y;
    p.angle += da + _rng.Gaussian(0, k5 * len + k6 * abs(da));

    if (_map.Intersects(p.loc, prev_loc)) {
      resample_vector.push_back(&p);
    }
  }

  _prev_odom_loc = odom_loc;
  _prev_odom_angle = odom_angle;

  for (Particle* p : resample_vector) {
    Resample(*p);
  }
}

void ParticleFilter::Resample(Particle& p) {
  p.loc = _prev_odom_loc;
  p.angle = _prev_odom_angle;
}

void ParticleFilter::Initialize(const string& map_file, const Vector2f& loc, const float angle) {
  _particles.clear();
  for (int i = 0; i < FLAGS_num_particles; ++i) {
    _particles.push_back(Particle{loc, angle, 1.0});
  }
  _map.Load(map_file);
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc, float* angle) const {}

} // namespace particle_filter

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
#include "navigation/navigation.h"

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

namespace particle_filter {

CONFIG_FLOAT(k1, "k1");
CONFIG_FLOAT(k2, "k2");
CONFIG_FLOAT(k3, "k3");
CONFIG_FLOAT(k4, "k4");
CONFIG_FLOAT(k5, "k5");
CONFIG_FLOAT(k6, "k6");
CONFIG_FLOAT(k_trans_scale, "k_trans_scale");
CONFIG_FLOAT(k_rot_scale, "k_rot_scale");

CONFIG_FLOAT(correlation, "correlation");
CONFIG_FLOAT(sigma, "sigma");
CONFIG_FLOAT(d_long, "d_long");
CONFIG_FLOAT(d_short, "d_short");
CONFIG_FLOAT(s_max_offset, "s_max_offset");
CONFIG_FLOAT(s_min_offset, "s_min_offset");
CONFIG_INT(stride, "stride");
config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter()
    : _prev_odom_loc(0, 0), _prev_odom_angle(0), _odom_initialized(false) {}

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = _particles;
}

void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc, const float angle, int num_ranges,
                                            float range_min, float range_max, float angle_min,
                                            float angle_max, float angle_increment, vector<Vector2f>* scan_ptr) {
  scan_ptr->clear();
  scan_ptr->reserve(num_ranges);

  static std::vector<float> predicted_ranges;
  predicted_ranges.clear();
  _map.GetPredictedScan(loc + Vector2f(cos(angle), sin(angle)) * 0.2, range_min, range_max, angle_min + angle, angle_max + angle, num_ranges, &predicted_ranges);

  for (int i = 0; i < num_ranges; ++i) {
    const float theta = angle + angle_min + (angle_increment * i);
    const Vector2f laser_pos = loc + Vector2f(navigation::LASER_OFFSET * cos(angle), navigation::LASER_OFFSET * sin(angle));
    float range = predicted_ranges[i];

    if (range > range_max + geometry::kEpsilon || range < range_min) {
      continue;
    }

    const float x = laser_pos.x() + range * cos(theta);
    const float y = laser_pos.y() + range * sin(theta);

    scan_ptr->push_back(Vector2f(x, y));
  }
}

float ParticleFilter::ray_cast(const Vector2f& loc, float angle, float max_range) {
  const Vector2f dir(cos(angle) * max_range, sin(angle) * max_range);
  static vector<geometry::line2f> lines;
  _map.GetSceneLines(loc, max_range, &lines);
  
  float sqdist_min = std::numeric_limits<float>().infinity();
  for (const geometry::line2f& line : lines) {
    float sqdist;
    Vector2f _;
    if (geometry::RayIntersect(loc, dir, line.p0, line.p1, sqdist, _) && sqdist < sqdist_min) {
      sqdist_min = sqdist;
    }
  }
  return sqrt(sqdist_min);
}

void ParticleFilter::Update(const vector<float>& ranges, float range_min, float range_max,
                            float angle_min, float angle_max, Particle* p_ptr) {
  const int stride = CONFIG_stride;
  const float sigma2 = CONFIG_sigma * CONFIG_sigma;
  const float d_long = CONFIG_d_long;
  const float d_short = CONFIG_d_short;

  const float gamma = (1.f - CONFIG_correlation) + (CONFIG_correlation / (ranges.size() / stride));
  const float s_min = range_min + CONFIG_s_min_offset;
  const float s_max = range_max - CONFIG_s_max_offset;

  Particle& particle = *p_ptr;
  static vector<float> predicted;
  predicted.clear();

  _map.GetPredictedScan(particle.loc + Vector2f(cos(particle.angle), sin(particle.angle)) * 0.2, range_min, range_max, angle_min + particle.angle, angle_max + particle.angle, ranges.size(), &predicted);
  
  float p = 0.f;
  for (uint i = 0; i < ranges.size(); i += stride) {
    double diff = 0.0;
    if (ranges[i] < s_min || ranges[i] > s_max) {
      continue;
    } else if (ranges[i] - predicted[i] < -d_short) {
      diff = pow(d_short, 2.0);
    } else if (ranges[i] - predicted[i] > d_long) {
      diff = pow(d_long, 2.0);
    } else {
      diff = pow(ranges[i] - predicted[i], 2.0);
    }

    p += -(diff / sigma2) * gamma;
  }
  particle.weight = p;
}

void ParticleFilter::Resample() {
  if (_particles.size() == 0) {
    return;
  }
  
  float w_max = _particles[0].weight;
  for (const Particle& p : _particles) {
    w_max = std::max(static_cast<double>(w_max), p.weight);
  }
  // std::cout << "w_max " << w_max << std::endl;

  for (Particle& p : _particles) {
    p.weight = exp(p.weight - w_max);
    // std::cout << p.weight << std::endl;
  }

  const uint n = _particles.size();
  float W = 0.f;
  for (const Particle& p : _particles) { W += p.weight; }

  const float x = _rng.UniformRandom(0, W);
  vector<Particle> resampled_particles;
  // std::cout << "W " << W << std::endl << std::endl;

  for (uint i = 0; i < n; ++i) {
    float w = 0;
    for (const Particle& p : _particles) {
      w += p.weight;

      // Low-variability resampling. Choose one rand number and other samples are at equidistant locations
      if (w > fmod(x + i * W / n, W)) {
        // std::cout << "REPLICATED" << std::endl;
        resampled_particles.emplace_back(p);
        break;
      }
    }
  }

  // This loop corrects any errors that result in a loss of overall particle population
  // We _should_ always have the same number of particles in these vectors, but this adds safety
  while (resampled_particles.size() < _particles.size()) {
    Particle p(Vector2f(), 0.f, 0.0); // weight will be adjusted on next pass of Update
    Resample(p); // set location and angle to our best guess + rand sample from loc and angle distributions

    resampled_particles.push_back(p);
  }
  
  _particles = resampled_particles;  
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges, float range_min, float range_max,
                                  float angle_min, float angle_max) {
  for (Particle& p : _particles) {
    Update(ranges, range_min, range_max, angle_min, angle_max, &p);
  }
  Resample();

  _location_dirty = true;
}

void ParticleFilter::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
  if (not _odom_initialized) {
    _prev_odom_loc = odom_loc;
    _prev_odom_angle = odom_angle;

    _odom_initialized = true;
    return;
  }

  const Vector2f dx = odom_loc - _prev_odom_loc;
  const Vector2f angle_vec(cos(odom_angle), sin(odom_angle));
  const double len = copysign(dx.norm(), dx.dot(angle_vec));

  const float da = math_util::AngleDiff(odom_angle, _prev_odom_angle);

  for (Particle& p : _particles) {
    const Vector2f prev_loc = p.loc;

    const Vector2f dir(cos(p.angle), sin(p.angle));
    const Vector2f perp_dir(cos(p.angle + M_PI / 2.f), sin(p.angle + M_PI / 2.f));

    const Vector2f e_x = dir * _rng.Gaussian(0, CONFIG_k1 * len + CONFIG_k2 * abs(da));
    const Vector2f e_y = perp_dir * _rng.Gaussian(0, CONFIG_k3 * len + CONFIG_k4 * abs(da));

    p.loc += (dir * len * CONFIG_k_trans_scale) + e_x + e_y;
    p.angle += da * CONFIG_k_rot_scale + _rng.Gaussian(0, CONFIG_k5 * len + CONFIG_k6 * abs(da));

    if (_map.Intersects(p.loc + dir * navigation::CAR_L, prev_loc)) {
      p.needs_resample = true;
    }
  }

  _prev_odom_loc = odom_loc;
  _prev_odom_angle = odom_angle;

  for (auto it = _particles.begin(); it != _particles.end();) {
    Particle& p = *it;
    if (p.needs_resample) {
      Resample(p);
      p.needs_resample = false;
      continue;
    }
    ++it;
  }
}

void ParticleFilter::Resample(Particle& p) {
  constexpr float pos_x_sigma = 0.1;
  constexpr float pos_y_sigma = 0.1;
  constexpr float angle_sigma = M_PI / 16.0;

  GetLocation(&p.loc, &p.angle);

  p.loc += Vector2f(_rng.Gaussian(0.f, pos_x_sigma), _rng.Gaussian(0.f, pos_y_sigma));
  p.angle += _rng.Gaussian(0.0, angle_sigma);
}

void ParticleFilter::Initialize(const string& map_file, const Vector2f& loc, const float angle) {
  constexpr float pos_x_sigma = 0.2;
  constexpr float pos_y_sigma = 0.2;
  constexpr float angle_sigma = M_PI / 8.0;
  _particles.clear();
  for (int i = 0; i < FLAGS_num_particles; ++i) {
    _particles.push_back(Particle(loc + Vector2f(_rng.Gaussian(0.f, pos_x_sigma), _rng.Gaussian(0.f, pos_y_sigma)),
                                  angle + _rng.Gaussian(0.0, angle_sigma), 1.0));
  }
  _odom_initialized = false; // fix for simulator
  _map.Load(map_file);
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc, float* angle) {
  if (_particles.size() != 0 && _location_dirty) {
    uint best_particle = 0;
    for (uint i = 0; i < _particles.size(); ++i) {
      const particle_filter::Particle& p = _particles[i];
      if (p.weight > _particles[best_particle].weight)
        best_particle = i;
    }
    const particle_filter::Particle& p = _particles[best_particle];
    _loc = p.loc;
    _angle = p.angle;
    _location_dirty = false;
  }

  *loc = _loc;
  *angle = _angle;
}

} // namespace particle_filter

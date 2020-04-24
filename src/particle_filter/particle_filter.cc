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
#include <array>
#include <cmath>
#include <iostream>

#include "config_reader/config_reader.h"
#include "navigation/navigation.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"

using Eigen::Vector2f;
using Eigen::Vector2i;
using std::array;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using vector_map::VectorMap;

DEFINE_double(num_particles, 50, "Number of particles");

namespace particle_filter {

CONFIG_BOOL(flag_location_smoothing, "flag_location_smoothing");
CONFIG_BOOL(flag_laser_smoothing, "flag_laser_smoothing");
CONFIG_BOOL(flag_variance_thresh, "flag_variance_thresh");
CONFIG_BOOL(flag_dist_update, "flag_dist_update");

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
CONFIG_FLOAT(s_max, "s_max");
CONFIG_FLOAT(s_min, "s_min");
CONFIG_INT(stride, "stride");

CONFIG_INT(resample_rate, "resample_rate");
CONFIG_FLOAT(update_dist, "update_dist");
CONFIG_FLOAT(var_threshold, "var_threshold");
CONFIG_FLOAT(location_smoothing, "location_smoothing");
config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter()
    : _prev_odom_loc(0, 0), _prev_odom_angle(0), _odom_initialized(false), _dist_since_update(0.f) {
}

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = _particles;
}

void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc, const float angle, int num_ranges,
                                            float range_min, float range_max, float angle_min,
                                            float angle_max, float angle_increment,
                                            vector<Vector2f>* scan_ptr) {
  scan_ptr->clear();
  scan_ptr->reserve(num_ranges);

  static std::vector<float> predicted_ranges;
  predicted_ranges.clear();
  _map.GetPredictedScan(loc + Vector2f(cos(angle), sin(angle)) * 0.2, range_min, range_max,
                        angle_min + angle, angle_max + angle, num_ranges, &predicted_ranges);

  for (int i = 0; i < num_ranges; ++i) {
    const float theta = angle + angle_min + (angle_increment * i);
    const Vector2f laser_pos = loc + Vector2f(navigation::LASER_OFFSET * cos(angle),
                                              navigation::LASER_OFFSET * sin(angle));
    float range = predicted_ranges[i];

    if (range > range_max + geometry::kEpsilon || range < range_min) {
      continue;
    }

    const float x = laser_pos.x() + range * cos(theta);
    const float y = laser_pos.y() + range * sin(theta);

    scan_ptr->push_back(Vector2f(x, y));
  }
}

// Unoptimized raycast function
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

void ParticleFilter::ConvolveGaussian(vector<float>& values) {
  if (!CONFIG_flag_laser_smoothing) {
    return;
  }

  constexpr static array<float, 7> kernel{0.071303, 0.131514, 0.189879, 0.214607,
                                          0.189879, 0.131514, 0.071303};
  static vector<float> src;
  src = values;

  for (ulong i = 0; i < values.size(); ++i) {
    values[i] = 0;
    for (ulong j = 0; j < kernel.size(); ++j) {
      const uint src_index = std::min(src.size(), std::max(0ul, i + j - kernel.size() / 2ul));
      values[i] += src[src_index] * kernel[j];
    }
  }
}

void ParticleFilter::Update(const vector<float>& ranges, float range_min, float range_max,
                            float angle_min, float angle_max, Particle* p_ptr) {
  const int stride =
      CONFIG_stride; // These values are set in the config file (config/particle_filter.lua)
  const float sigma2 = CONFIG_sigma * CONFIG_sigma;
  const float d_long = CONFIG_d_long;
  const float d_short = CONFIG_d_short;

  const float gamma = (1.f - CONFIG_correlation) + (CONFIG_correlation / (ranges.size() / stride));
  const float s_min = CONFIG_s_min;
  const float s_max = CONFIG_s_max;

  Particle& particle = *p_ptr;
  static vector<float> predicted;
  predicted.clear();

  _map.GetPredictedScan(particle.loc + Vector2f(cos(particle.angle), sin(particle.angle)) * 0.2,
                        range_min, range_max, angle_min + particle.angle,
                        angle_max + particle.angle, ranges.size(), &predicted);
  ConvolveGaussian(predicted);

  // Robust observation likelihood model
  float p = 0.f;
  for (uint i = 0; i < ranges.size(); i += stride) {
    double diff = 0.0;
    if (ranges[i] < s_min || ranges[i] > s_max) {
      continue;
    } else if (ranges[i] - predicted[i] < -d_short) {
      // There's something unexpected closer than the wall
      diff = pow(d_short, 2);
    } else if (ranges[i] - predicted[i] > d_long) {
      // There's something unexpected beyond the wall
      diff = pow(d_long, 2);
    } else {
      diff = pow(abs(ranges[i] - predicted[i]), 2);
    }

    p += -(diff / sigma2) * gamma;
  }
  particle.weight = p;
}

void ParticleFilter::Resample() {
  if (_particles.size() == 0) {
    return;
  }

  // Find max weight
  float w_max = _particles[0].weight;
  for (const Particle& p : _particles) {
    w_max = std::max(static_cast<double>(w_max), p.weight);
  }

  // Normalize particle weights using the max
  for (Particle& p : _particles) {
    p.weight = exp(p.weight - w_max);
  }

  const uint n = _particles.size();
  float W = 0.f;
  for (const Particle& p : _particles) {
    W += p.weight;
  }

  const float x = _rng.UniformRandom(0, W);
  vector<Particle> resampled_particles;

  for (uint i = 0; i < n; ++i) {
    float w = 0;
    for (const Particle& p : _particles) {
      w += p.weight;

      // Low-variance resampling. Choose one rand number and other samples are at equidistant
      // locations
      if (w > fmod(x + i * W / n, W)) {
        resampled_particles.emplace_back(p);
        break;
      }
    }
  }

  // This loop corrects any errors that result in a loss of overall particle population
  // We _should_ always have the same number of particles in these vectors, but this adds safety
  while (resampled_particles.size() < _particles.size()) {
    Particle p(Vector2f(), 0.f, 0.0); // weight will be adjusted on next pass of Update
    Resample(p); // set location and angle to our best guess + rand sample from loc and angle
                 // distributions

    resampled_particles.push_back(p);
  }

  _particles = resampled_particles;
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges, float range_min, float range_max,
                                  float angle_min, float angle_max) {
  static uint frame_counter = 0;

  if (CONFIG_flag_dist_update && _dist_since_update < CONFIG_update_dist) {
    return;
  }

  _dist_since_update = 0.f;

  static vector<float> ranges_smoothed;
  ranges_smoothed = ranges;
  ConvolveGaussian(ranges_smoothed);

  double mean_weight = 0; // Find for calculating the weight variance
  for (Particle& p : _particles) {
    Update(ranges_smoothed, range_min, range_max, angle_min, angle_max, &p);
    mean_weight += p.weight;
  }
  mean_weight /= _particles.size();

  double var = 0.f;
  for (Particle& p : _particles) {
    var += pow(p.weight - mean_weight, 2.0);
  }
  var = var / _particles.size();

  ++frame_counter;

  // If we are on a "resample frame" or our variance is above the threshold, resample
  if (frame_counter % CONFIG_resample_rate == 0 || var > CONFIG_var_threshold) {
    Resample();
  }

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
  _dist_since_update += dx.norm();

  const Vector2f angle_vec(cos(odom_angle), sin(odom_angle));
  const double len = copysign(dx.norm(), dx.dot(angle_vec));

  const float da = math_util::AngleDiff(odom_angle, _prev_odom_angle);

  // Motion model
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

  // Replace particles that hit walls
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

// Resamples a single particle to somewhere around our best location hypothesis
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
    _particles.push_back(
        Particle(loc + Vector2f(_rng.Gaussian(0.f, pos_x_sigma), _rng.Gaussian(0.f, pos_y_sigma)),
                 angle + _rng.Gaussian(0.0, angle_sigma), 1.0));
  }
  _odom_initialized = false; // fix for simulator
  _map.Load(map_file);
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc, float* angle) {
  if (_particles.size() == 0 || !_location_dirty) {
    *loc = _loc;
    *angle = _angle;
    return;
  }

  // Location is dirty. Let's recompute it here
  uint best_particle = 0;
  for (uint i = 0; i < _particles.size(); ++i) {
    const particle_filter::Particle& p = _particles[i];
    if (p.weight > _particles[best_particle].weight) {
      best_particle = i;
    }
  }

  const particle_filter::Particle& p = _particles[best_particle];
  _loc = p.loc;
  _angle = p.angle;

  // Smoothed location
  _loc_smoothed =
      p.loc * (1 - CONFIG_location_smoothing) + _loc_smoothed * CONFIG_location_smoothing;
  const Vector2f new_angle_vec(cos(_angle_smoothed) * CONFIG_location_smoothing +
                                   cos(p.angle) * (1 - CONFIG_location_smoothing),
                               sin(_angle_smoothed) * CONFIG_location_smoothing +
                                   sin(p.angle) * (1 - CONFIG_location_smoothing));
  _angle_smoothed = atan2(new_angle_vec.y(), new_angle_vec.x());

  _location_dirty = false;
  *loc = _loc;
  *angle = _angle;
}

void ParticleFilter::GetSmoothedLocation(Eigen::Vector2f* loc, float* angle) {
  // Calling GetLocation checks to see if location is dirty, and recomputes it if so
  GetLocation(loc, angle);

  if (CONFIG_flag_location_smoothing) {
    *loc = _loc_smoothed;
    *angle = _angle_smoothed;
  }
}

} // namespace particle_filter

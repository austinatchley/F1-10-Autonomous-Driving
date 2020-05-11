#include "RRT.h"

#include <cmath>
#include <deque>
#include <unordered_map>

#include "config_reader/config_reader.h"
#include "shared/math/geometry.h"
#include "shared/util/random.h"
#include "visualization/visualization.h"

using Eigen::Vector2f;
using std::string;

namespace planning {
CONFIG_INT(rrt_max_iter_frame, "rrt_max_iter_frame");
CONFIG_INT(rrt_min_total_iter, "rrt_min_total_iter");
CONFIG_INT(rrt_max_iter, "rrt_max_iter");
CONFIG_FLOAT(rrt_goal_tolerance, "rrt_goal_tolerance");
CONFIG_FLOAT(rrt_wall_dilation, "rrt_wall_dilation");
CONFIG_FLOAT(rrt_neighborhood_radius, "rrt_neighborhood_radius");
CONFIG_FLOAT(rrt_steering_eta, "rrt_steering_eta");
config_reader::ConfigReader config_reader_({"config/navigation.lua"});

void RRT::Initialize() {
  // TODO: Flood fill on init and compute the practical max/min,
  // as opposed to the max/min defined by the map
  for (const geometry::Line<float>& line : _map.lines) {
    _map_min = _map_min.cwiseMin(line.p0);
    _map_min = _map_min.cwiseMin(line.p1);
    _map_max = _map_max.cwiseMax(line.p0);
    _map_max = _map_max.cwiseMax(line.p1);
  }
}

void RRT::StartFindPath(const Vector2f& start, const Vector2f& goal) {
  _start = start;
  _goal = goal;
  _pathfinding = true;
  _Reset();
}

void RRT::_Reset() {
  _vertices.clear();
  _vertices.push_back(Vertex(_start, 0.f));
  
  _vertex_grid.clear();
  _total_iter = 0;
  _has_path_to_goal = false;
  _best_end_vertex = nullptr;
}

bool RRT::IsFindingPath() {
  return _pathfinding;
}

bool RRT::FindPath(std::deque<Vertex>& path, int& i) {
  if (!_pathfinding) {
    throw std::runtime_error("Call StartFindPath() before FindPath()!");
  }

  path.clear();

  bool finished_pathfinding = false;
  i = 0;
  while (true) {
    if (_total_iter + i >= CONFIG_rrt_max_iter) {
      // hit total max iteration limit; reset tree and try again later
      _Reset(); 
      break;
    }
    if (i >= CONFIG_rrt_max_iter_frame) {
      break;
    }

    Vertex x_rand;
    if (_has_path_to_goal) {
      x_rand = Sample(SelectBestEndVertex().cost);
    } else {
      x_rand = Sample(std::numeric_limits<float>().max());
    }
    visualization::DrawPoint(x_rand.loc, 0xFF0069, _msg);

    Vertex& x_nearest = Nearest(x_rand);
    Vertex x_new_stack = Steer(x_nearest, x_rand); // version of x_new that goes on the stack and will be recycled

    if (ObstacleFree(x_nearest, x_new_stack)) {
      _vertices.push_back(x_new_stack);
      Vertex& x_new = _vertices.back(); // version of x_new that gets managed by vector

      // Push a pointer to the vertex into our acceleration structure
      _vertex_grid[WorldToGrid(x_new.loc)].push_back(&x_new);

      std::vector<Vertex*> near;
      GetNeighbors(x_new, near, CONFIG_rrt_neighborhood_radius);

      Vertex* x_min = &x_nearest;
      float c_min = x_nearest.cost + Cost(x_nearest, x_new);
      for (Vertex* x_near : near) {
        if (ObstacleFree(*x_near, x_new)) {
          const float c = x_near->cost + Cost(*x_near, x_new);

          if (c < c_min) {
            x_min = x_near;
            c_min = c;
          }
        }
      }
      x_new.parent = x_min;
      x_new.cost = x_min->cost + Cost(*x_min, x_new);

      // Rewire to find lowest cost path
      for (Vertex* x_near : near) {
        if (x_near == x_min) {
          continue;
        }

        const float c = x_new.cost + Cost(x_new, *x_near);
        if (ObstacleFree(x_new, *x_near) && c < x_near->cost) {
          x_near->parent = &x_new;
          x_near->cost = c;
        }
      }

      const bool reached_goal = ReachedGoal(x_new, _goal);
      if (reached_goal) {
        _has_path_to_goal = true;

        // Add the final segment to the cost
        x_new.cost += Cost(x_new, Vertex(_goal));

        if (!_best_end_vertex || x_new.cost < _best_end_vertex->cost) {
          _best_end_vertex = &x_new;
        }
      }
      if (_total_iter >= CONFIG_rrt_min_total_iter && reached_goal) {
        finished_pathfinding = true;
        break;
      }
    }
    ++i;
    ++_total_iter;
  }
  std::cerr << "RRT* iters: " << _total_iter << "/" << CONFIG_rrt_min_total_iter;
  std::cerr << " (" << CONFIG_rrt_max_iter << " max)" << std::endl;

  // reconstruct best path
  Vertex& best_vertex = SelectBestEndVertex();
  Vertex* current = &best_vertex;
  path.push_front(*current);
  while (current->parent) {
    current = current->parent;
    path.push_front(*current);
  }

  if (finished_pathfinding) {
    _pathfinding = false;
  }
  return finished_pathfinding;
}

Vertex& RRT::SelectBestEndVertex() {
  if (_best_end_vertex) {
    return *_best_end_vertex;
  }

  // otherwise select vertex nearest to goal
  return Nearest(Vertex(_goal));
}

Vertex RRT::Sample(const float c_max) {
  static util_random::Random rng;
  if (c_max >= std::numeric_limits<float>().max()) {
    return Vertex({rng.UniformRandom(_map_min.x(), _map_max.x()),
                   rng.UniformRandom(_map_min.y(), _map_max.y())});
  }
  const float c_min = (_goal - _start).norm();
  const Vector2f x_center = (_start + _goal) / 2.f;

  const float angle = atan2(_goal.y() - _start.y(), _goal.x() - _start.x());
  const Rotation2D<float> rot_to_world(angle);

  // Scale ellipse axes
  const Vector2f L_x_ball = SampleUnitNBall().cwiseProduct(Vector2f(
    c_max / 2.f,
    sqrt(fmax(0.f, pow(c_max, 2) - pow(c_min, 2))) / 2.f
  ));
  
  // we allow samples outside of map bounds
  Vertex ret(rot_to_world * L_x_ball + x_center);
  return ret;
}

Vector2f RRT::SampleUnitNBall() {
  static util_random::Random rng;
  Vector2f rand;
  do {  
    rand = Vector2f(rng.UniformRandom(-1, 1), rng.UniformRandom(-1, 1));
  } while (rand.norm() > 1);
  return rand;
}

Vertex& RRT::Nearest(const Vertex& x) {
  if (_vertices.size() == 0) {
    throw std::runtime_error("vertices empty in Nearest()!");
  }
  float min_dist = std::numeric_limits<float>().max();
  int min = 0;
  for (uint i = 0; i < _vertices.size(); ++i) {
    const Vertex& v = _vertices.at(i);
    const float dist = v.distance(x);
    if (dist < min_dist) {
      min = i;
      min_dist = dist;
    }
  }

  return _vertices.at(min);
}

bool RRT::ReachedGoal(const Vertex& pos, const Vertex& goal) {
  const float delta = CONFIG_rrt_goal_tolerance;
  return pos.distance(goal) < delta;
}

bool RRT::ObstacleFree(const Vertex& x0, const Vertex& x1) {
  const float min_dist = CONFIG_rrt_wall_dilation;

  Vector2f min_a(std::min(x0.loc.x(), x1.loc.x()) - min_dist,
                 std::min(x0.loc.y(), x1.loc.y()) - min_dist);
  Vector2f max_a(std::max(x0.loc.x(), x1.loc.x()) + min_dist,
                 std::max(x0.loc.y(), x1.loc.y()) + min_dist);

  for (const geometry::Line<float>& line : _map.lines) {
    Vector2f min_b(std::min(line.p0.x(), line.p1.x()) - min_dist,
                   std::min(line.p0.y(), line.p1.y()) - min_dist);
    Vector2f max_b(std::max(line.p0.x(), line.p1.x()) + min_dist,
                   std::max(line.p0.y(), line.p1.y()) + min_dist);

    // Broad phase check
    if (min_a.x() > max_b.x() || min_a.y() > max_b.y() || max_a.x() < min_b.x() ||
        max_a.y() < min_b.y()) {
      continue;
    }
    if (geometry::MinDistanceLineLine(line.p0, line.p1, x0.loc, x1.loc) < min_dist) {
      return false;
    }
  }
  return true;
}

Vertex RRT::Steer(const Vertex& x0, const Vertex& x1) {
  const float eta = CONFIG_rrt_steering_eta;
  const Vector2f dx = x1.loc - x0.loc;
  return Vertex(x0.loc + dx.normalized() * std::min(eta, dx.norm()), 0.f);
}

void RRT::GetNeighbors(const Vertex& x, std::vector<Vertex*>& neighbors, const float neighborhood_radius) {
  // find extrema using neighbohood radius
  const Vector2i min_cell = WorldToGrid(x.loc - Vector2f(neighborhood_radius, neighborhood_radius));
  const Vector2i max_cell = WorldToGrid(x.loc + Vector2f(neighborhood_radius, neighborhood_radius));

  // iterate over rectangular region of grid cells, add to vector
  for (int i = min_cell.x(); i <= max_cell.x(); ++i) {
    for (int j = min_cell.y(); j <= max_cell.y(); ++j) {
      for (Vertex* v : _vertex_grid[Vector2i(i, j)]) {
        if (x.loc == v->loc) continue;

        if (x.distance(*v) < neighborhood_radius) {
          neighbors.push_back(v);
        }
      }
    }
  }
}

void RRT::GetNaiveNeighbors(const Vertex& x, std::vector<Vertex*>& neighbors, const float neighborhood_radius) {
  for (Vertex& other : _vertices) {
    if (x.loc == other.loc) continue;
    if (x.distance(other) < neighborhood_radius) {
      neighbors.push_back(&other);
    }
  }
}

float RRT::Cost(const Vertex& x0, const Vertex& x1) {
  // TODO: Consider angle here

  return (x0.loc - x1.loc).norm();
}

void RRT::VisualizePath(std::deque<Vertex>& path) {
  for (uint i = 1; i < path.size(); ++i) {
    visualization::DrawLine(path[i].loc, path[i - 1].loc, 0x69AF00, _msg);
  }
}

void RRT::VisualizeCurrentTree() {
  for (const Vertex& v : _vertices) {
    if (v.parent == nullptr) continue;
    visualization::DrawLine(v.loc, v.parent->loc, 0x555555, _msg);
  }
}

Vector2i RRT::WorldToGrid(const Vector2f& world) {
  return Vector2i(std::floor(world.x() / _grid_size), std::floor(world.y() / _grid_size));
}

Vector2f RRT::GridToWorld(const Vector2i& grid) {
  return Vector2f(grid.x() * _grid_size, grid.y() * _grid_size);
}

} // namespace planning
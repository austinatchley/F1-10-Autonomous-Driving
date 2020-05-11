#include <deque>
#include <unordered_map>

#include "Vertex.h"
#include "f1tenth_course/VisualizationMsg.h"
#include "vector_map/vector_map.h"

using std::string;

#ifndef RRT_H
#define RRT_H

namespace std {
template <>
struct hash<Eigen::Vector2i> {
  std::size_t operator()(Eigen::Vector2i const& s) const noexcept {
    return std::hash<int>()(s.x()) ^ (std::hash<int>()(s.y()) << 1);
  }
};
} // namespace std

namespace planning {
using namespace Eigen;
using VertexGrid = std::unordered_map<Vector2i, std::vector<Vertex*>>;

class RRT {
public:
  RRT(const vector_map::VectorMap& map, f1tenth_course::VisualizationMsg& msg)
      : _map(map), _msg(msg) {}

  // Set map bounds and init misc values
  void Initialize();

  // Initialize path finding state for calls to FindPath
  void StartFindPath(const Vector2f& start, const Vector2f& goal);

  // Returns true if StartFindPath has been called and we have not reached success in FindPath
  bool IsFindingPath();

  // Finds a path between cur and goal using RRT*
  // Results are improved upon successive calls if success was not reached (hit iteration limit)
  bool FindPath(std::deque<Vertex>& path, int& i);

  // Choose the vertex with minimum-cost path to goal, or the closest vertex to goal
  // if no vertices have reached the goal yet.
  Vertex& SelectBestEndVertex();

  // Returns true if we are within reasonable distance of the provided goal
  bool ReachedGoal(const Vertex& pos, const Vertex& goal);

  // Returns true if the line connecting the two given points does not collide with any objects
  bool ObstacleFree(const Vertex& x0, const Vertex& x1);

  // sample a point for informed RRT*
  Vertex Sample(const float c_max);
  Vector2f SampleUnitNBall();

  // Returns a point between x0 and x1
  Vertex Steer(const Vertex& x0, const Vertex& x1);

  // Returns the nearest vertex to x
  Vertex& Nearest(const Vertex& x);

  // Returns a vector of points to the vertices representing the neighbors of a given point
  // Uses spatially hashed grid
  void GetNeighbors(const Vertex& x, std::vector<Vertex*>& neighbors, const float neighborhood_radius);

  void GetNNearestNeighbors(const Vertex& x, std::vector<Vertex*>& neighbors, const unsigned int n);
  void GetNNearestNeighborsFast(const Vertex& x, std::vector<Vertex*>& neighbors, const unsigned int n);

  // Returns a vector of points to the vertices representing the neighbors of a given point
  // Uses naive distance formula approach
  void GetNaiveNeighbors(const Vertex& x, std::vector<Vertex*>& neighbors, const float neighborhood_radius);

  // Returns the result of the edge cost function (i.e. Euclidean distance between points)
  float Cost(const Vertex& x0, const Vertex& x1);

  // Draw path lines to _msg
  void VisualizePath(std::deque<Vertex>& path);
  void VisualizeCurrentTree();

  // Convert world coordinates to grid coordinates for spatial hashing grid
  Vector2i WorldToGrid(const Vector2f& world);

  // Convert grid coordinates to world coordinates for going backwards from spatial hashing grid
  Vector2f GridToWorld(const Vector2i& grid);

private:
  // clear the tree and insert start node
  void _Reset();

  bool _pathfinding = false;
  bool _has_path_to_goal = false;
  VertexGrid _vertex_grid;
  std::deque<Vertex> _vertices;
  Vector2f _map_min, _map_max;
  Vector2f _start;
  Vector2f _goal;
  Vertex* _best_end_vertex = nullptr;

  int _total_iter;
  int _total_iter_first_path;

  float _grid_size = 1.f;

  const vector_map::VectorMap& _map;
  f1tenth_course::VisualizationMsg& _msg;
};
} // namespace planning

#endif // RRT_H
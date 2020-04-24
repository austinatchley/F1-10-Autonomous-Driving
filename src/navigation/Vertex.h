#include "eigen3/Eigen/Dense"

namespace planning {
using namespace Eigen;

struct Vertex {
  Vertex() : loc(0.f, 0.f), cost(0.f) {}
  Vertex(const Vector2f& loc) : loc(loc), cost(0.f) {}
  Vertex(const Vector2f& loc, float cost) : loc(loc), cost(cost) {}

  double distance(const Vertex& other) const {
    return (loc - other.loc).norm();
  }

  Vertex* parent = nullptr;
  Vector2f loc;
  float cost;

  // float _angle;
};
} // namespace planning
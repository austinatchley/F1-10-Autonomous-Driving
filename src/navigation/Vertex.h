#include "eigen3/Eigen/Dense"

namespace planning {
using namespace Eigen;

struct Vertex {
  Vertex() : loc(0.f, 0.f) {}
  Vertex(const Vector2f& loc) : loc(loc) {}

  double distance(const Vertex& other) const {
    return (loc - other.loc).norm();
  }

  Vertex* parent = nullptr;
  Vector2f loc;
  
  float cost() const {
    if (!parent) { return 0.f; }

    return distance(*parent) + parent->cost();
  }

  // float _angle;
};
} // namespace planning
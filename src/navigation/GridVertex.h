#include "eigen3/Eigen/Dense"

using Eigen::Vector2f;

namespace planning {
class GridVertex {
public:
    GridVertex() : _pos(0.f, 0.f) {}
    GridVertex(const Vector2f& pos) : _pos(pos) {}

    // TODO: Add more functionality here based on the structure of our grid

private:
    Vector2f _pos;
};
} // namespace planning
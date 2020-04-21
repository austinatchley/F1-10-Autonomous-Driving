#include "Pose.h"

namespace planning {
class Vertex {
public:
    Vertex() : _pose(Vector2f(), 0.f), _cost(0.f) {}
    Vertex(const Pose& pose) : _pose(pose), _cost(0.f) {}
    Vertex(const Pose& pose, float cost) : _pose(pose), _cost(cost) {}

    Vector2f GetPos() { return _pose.pos; }

    // TODO: Add more functionality here based on the structure of our grid

private:
    Pose _pose;
    float _cost;
};
} // namespace planning
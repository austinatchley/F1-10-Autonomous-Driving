#include "Pose.h"

namespace planning {
class Vertex {
public:
    Vertex() : _pose(Vector2f(), 0.f) {}
    Vertex(const Pose& pose) : _pose(pose) {}

    Vector2f GetPos() { return _pose.pos; }

    // TODO: Add more functionality here based on the structure of our grid

private:
    Pose _pose;
};
} // namespace planning
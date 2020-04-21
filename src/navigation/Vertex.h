#include "eigen3/Eigen/Dense"

namespace planning {
using namespace Eigen;

class Vertex {
public:
    Vertex() : loc(0.f, 0.f), _cost(0.f) {}
    Vertex(const Vector2f& loc) : loc(loc), _cost(0.f) {}
    Vertex(const Vector2f& loc, float cost) : loc(loc), _cost(cost) {}
    double distance(const Vertex& other) {
        return (loc - other.loc).norm();
    }

    Vertex* parent = nullptr;
    Vector2f loc;
private:
    float _cost;
    
    // float _angle;
};
} // namespace planning
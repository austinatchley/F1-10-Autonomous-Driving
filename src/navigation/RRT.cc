#include <vector>
#include "RRT.h"

#include "shared/util/random.h"

using std::string;
using Eigen::Vector2f;

namespace planning {

void RRT::Initialize() {
    for (const geometry::Line<float>& line : _map.lines) {
        _map_min.cwiseMin(line.p0);
        _map_min.cwiseMin(line.p1);
        _map_max.cwiseMax(line.p0);
        _map_max.cwiseMax(line.p1);
    }
}

void RRT::FindPath(const Vector2f& cur, const Vector2f& goal, std::vector<Vertex>& path) {
    static constexpr int N = 500;
    static util_random::Random rng;
    _goal = goal;
    path.clear();

    std::vector<Vertex> vertices;
    vertices.push_back(Vertex(cur, 0.f));

    for (int i = 0; i < N; ++i) {
        const Vector2f x_rand{
            rng.UniformRandom(_map_min.x(), _map_max.x()), 
            rng.UniformRandom(_map_min.y(), _map_max.y())
        };

    }
}

bool RRT::ReachedGoal(const Vertex& pos, const Vertex& goal) {
    return false;
}

bool RRT::ObstacleFree(const Vertex& x1, const Vertex& x2) {
    return false;
}

Vertex RRT::Steer(const Vertex& x1, const Vertex& x2) {
    return Vertex(Vector2f(), 0.f);
}
} // namespace planning
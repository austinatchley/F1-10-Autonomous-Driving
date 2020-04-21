#include <deque>
#include "RRT.h"

#include "shared/util/random.h"
#include "visualization/visualization.h"

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

void RRT::FindPath(const Vector2f& cur, const Vector2f& goal, std::deque<Vertex>& path) {
    static constexpr int N = 500;
    static util_random::Random rng;
    path.clear();

    std::deque<Vertex> vertices;
    vertices.push_back(Vertex(cur, 0.f));

    for (int i = 0; i < N; ++i) {
        const Vertex x_rand{{
            rng.UniformRandom(_map_min.x(), _map_max.x()), 
            rng.UniformRandom(_map_min.y(), _map_max.y())
        }};

        Vertex& x_nearest = Nearest(x_rand, vertices);
        Vertex x_new = Steer(x_nearest, x_rand);

        if (ObstacleFree(x_nearest, x_new)) {
            x_new.parent = &x_nearest;
            vertices.push_back(x_new);
        }

        if (ReachedGoal(x_new, goal)) {
            break;
        }
    }
    
    Vertex& nearest = Nearest(Vertex(goal), vertices);
    Vertex* current = &nearest;
    path.push_front(*current);
    while (current->parent) {
        current = current->parent;
        path.push_front(*current);
    }
}

Vertex& RRT::Nearest(const Vertex& x, std::deque<Vertex>& vertices) {
    float min_dist = std::numeric_limits<float>().max();
    int min = 0;
    for (uint i = 0; i < vertices.size(); ++i) {
        const Vertex& v = vertices[i];
        const float dist = v.distance(x);
        if (dist < min_dist) {
            min = i;
            min_dist = dist;
        }
    }

    return vertices[min];
}

bool RRT::ReachedGoal(const Vertex& pos, const Vertex& goal) {
    static constexpr float delta = 0.1;

    return pos.distance(goal) < delta;
}

bool RRT::ObstacleFree(const Vertex& x0, const Vertex& x1) {
    return _map.Intersects(x0.loc, x1.loc);
}

Vertex RRT::Steer(const Vertex& x0, const Vertex& x1) {
    static constexpr float eta = 1.f;
    return Vertex(x0.loc + (x1.loc - x0.loc).normalized() * eta, 0.f);
}

void RRT::VisualizePath(std::deque<Vertex>& path) {
    for (const Vertex& v : path) {
        if (v.parent == nullptr) { continue; }

        visualization::DrawLine(v.loc, v.parent->loc, 0xAF00AF, _msg);
    }
} 

} // namespace planning
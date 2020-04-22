#include "RRT.h"

#include <deque>
#include <cmath>
#include <unordered_map>

#include "shared/util/random.h"
#include "shared/math/geometry.h"
#include "visualization/visualization.h"
#include "config_reader/config_reader.h"

using std::string;
using Eigen::Vector2f;

namespace planning {
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

bool RRT::FindPath(const Vector2f& cur, const Vector2f& goal, std::deque<Vertex>& path, size_t& i) {
    const size_t N = CONFIG_rrt_max_iter;
    static util_random::Random rng;
    path.clear();

    std::deque<Vertex> vertices;
    vertices.push_back(Vertex(cur, 0.f));

    // TODO: Implement jump-point or Informed RRT*

    // Grid for spatial hashing
    VertexGrid vertex_grid;

    for (i = 0; i < N; ++i) {
        const Vertex x_rand{{
            rng.UniformRandom(_map_min.x(), _map_max.x()), 
            rng.UniformRandom(_map_min.y(), _map_max.y())
        }};

        // visualization::DrawPoint(x_rand.loc, 0x007000, _msg);
        Vertex& x_nearest = Nearest(x_rand, vertices);
        Vertex x_new_stack = Steer(x_nearest, x_rand);

        // std::cout << "(" << x_rand.loc.x() << ", " << x_rand.loc.y() << ") -> (" << x_new.loc.x() << ", " << x_new.loc.y() << std::endl;
        // std::cout << (ObstacleFree(x_nearest, x_new) ? "FREE" : "BLOCKED") << std::endl;

        if (ObstacleFree(x_nearest, x_new_stack)) {
            vertices.push_back(x_new_stack);
            Vertex& x_new = vertices.back();

            // Push a pointer to the vertex into our acceleration structure
            vertex_grid[WorldToGrid(x_new.loc)].push_back(&x_new);

            std::vector<Vertex*> near;
            GetNeighbors(vertex_grid, vertices, x_new, near);

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
            for (Vertex* x_near : near) {
                if (x_near == x_min) { continue; }

                const float c = x_new.cost + Cost(x_new, *x_near);
                if (ObstacleFree(x_new, *x_near) && c < x_near->cost) {
                    x_near->parent = &x_new;
                    x_near->cost = c;
                }
            }
        
            // TODO: Can we keep iterating on the path instead of just breaking here?
            if (ReachedGoal(x_new_stack, goal)) {
                break;
            }
        }
 
    }
    std::cerr << "RRT* ITERS: " << i << std::endl;

    for (const Vertex& v : vertices) {
        if (v.parent == nullptr)
            continue;
        visualization::DrawLine(v.loc, v.parent->loc, 0x555555, _msg);
    }
    
    Vertex& nearest = Nearest(Vertex(goal), vertices);
    Vertex* current = &nearest;
    path.push_front(*current);
    while (current->parent) {
        current = current->parent;
        path.push_front(*current);
    }

    return i < N;
}

void RRT::FindNaivePath(const Vector2f& cur, const Vector2f& goal, std::deque<Vertex>& path) {
    const int N = CONFIG_rrt_max_iter;
    static util_random::Random rng;
    path.clear();

    std::deque<Vertex> vertices;
    vertices.push_back(Vertex(cur, 0.f));

    for (int i = 0; i < N; ++i) {
        const Vertex x_rand{{
            rng.UniformRandom(_map_min.x(), _map_max.x()), 
            rng.UniformRandom(_map_min.y(), _map_max.y())
        }};

        // visualization::DrawPoint(x_rand.loc, 0x007000, _msg);
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

    for (const Vertex& v : vertices) {
        if (v.parent == nullptr)
            continue;
        visualization::DrawLine(v.loc, v.parent->loc, 0x404040, _msg);
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
    if (vertices.size() == 0) {
        throw std::runtime_error("vertices empty in Nearest()!");
    }
    float min_dist = std::numeric_limits<float>().max();
    int min = 0;
    for (uint i = 0; i < vertices.size(); ++i) {
        const Vertex& v = vertices.at(i);
        const float dist = v.distance(x);
        if (dist < min_dist) {
            min = i;
            min_dist = dist;
        }
    }

    // std::cout << "nearest loc: " << vertices.at(min).loc << std::endl;

    return vertices.at(min);
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
        if (min_a.x() > max_b.x() || min_a.y() > max_b.y() ||
            max_a.x() < min_b.x() || max_a.y() < min_b.y()) {
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

void RRT::GetNeighbors(VertexGrid& vertex_grid, std::deque<Vertex>& vertices, const Vertex& x, std::vector<Vertex*>& neighbors) {
    const float neighborhood_radius = CONFIG_rrt_neighborhood_radius; 

    // find extrema using neighbohood radius
    const Vector2i min_cell = WorldToGrid(x.loc - Vector2f(neighborhood_radius, neighborhood_radius));
    const Vector2i max_cell = WorldToGrid(x.loc + Vector2f(neighborhood_radius, neighborhood_radius));

    // iterate over rectangular region of grid cells, add to vector
    for (int i = min_cell.x(); i <= max_cell.x(); ++i) {
        for (int j = min_cell.y(); j <= max_cell.y(); ++j) {
            for (Vertex* v : vertex_grid[Vector2i(i, j)]) {
                if (x.loc == v->loc)
                    continue;

                if (x.distance(*v) < neighborhood_radius) {
                    neighbors.push_back(v);
                }
            }
        }
    }
}

void RRT::GetNaiveNeighbors(std::deque<Vertex>& vertices, const Vertex& x, std::vector<Vertex*>& neighbors) {
    const float neighborhood_radius = CONFIG_rrt_neighborhood_radius; 

    for (Vertex& other : vertices) {
        if (x.loc == other.loc)
            continue;
        if (x.distance(other) < neighborhood_radius) {
            neighbors.push_back(&other);
        }
    }
}

float RRT::Cost(const Vertex& x0, const Vertex& x1) {
    // TODO: Consider angle here

    return (x0.loc - x1.loc).squaredNorm();
}

void RRT::VisualizePath(std::deque<Vertex>& path) {
    for (uint i = 1; i < path.size(); ++i) {
        visualization::DrawLine(path[i].loc, path[i-1].loc, 0x69AF00, _msg);
    }
}

Vector2i RRT::WorldToGrid(const Vector2f& world) {
    return Vector2i(std::floor(world.x() / _grid_size),
                    std::floor(world.y() / _grid_size));
}

Vector2f RRT::GridToWorld(const Vector2i& grid) {
    return Vector2f(grid.x() * _grid_size,
                    grid.y() * _grid_size);
}

} // namespace planning
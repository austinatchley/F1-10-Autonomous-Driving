#include <deque>
#include <unordered_map>

#include "f1tenth_course/VisualizationMsg.h"
#include "Vertex.h"
#include "vector_map/vector_map.h"

using std::string;

#ifndef RRT_H
#define RRT_H

namespace std {
    template<> struct hash<Eigen::Vector2i> {
        std::size_t operator()(Eigen::Vector2i const& s) const noexcept {
            return std::hash<int>()(s.x()) ^ (std::hash<int>()(s.y()) << 1);
        }
    };
}

namespace planning { 
using namespace Eigen;
using VertexGrid = std::unordered_map<Vector2i, std::vector<Vertex*>>;

class RRT {
public:
    RRT(const vector_map::VectorMap& map, f1tenth_course::VisualizationMsg& msg) : _map(map), _msg(msg) {}

    // Set map bounds and init misc values
    void Initialize();
    void StartFindPath(const Vector2f& cur, const Vector2f& goal);

    // Finds a path between cur and goal using RRT*
    bool FindPath(std::deque<Vertex>& path, size_t& i);

    // Finds a path between cur and goal using basic RRT
    void FindNaivePath(std::deque<Vertex>& path);

    // Returns true if we are within reasonable distance of the provided goal
    bool ReachedGoal(const Vertex& pos, const Vertex& goal);

    // Returns true if the line connecting the two given points does not collide with any objects
    bool ObstacleFree(const Vertex& x0, const Vertex& x1);

    // Returns a point between x0 and x1
    Vertex Steer(const Vertex& x0, const Vertex& x1);

    Vertex& Nearest(const Vertex& x);

    // Returns a vector of points to the vertices representing the neighbors of a given point
    void GetNeighbors(const Vertex& x, std::vector<Vertex*>& neighbors);
    void GetNaiveNeighbors(const Vertex& x, std::vector<Vertex*>& neighbors);

    // Returns the result of the edge cost function (i.e. Euclidean distance between points)
    float Cost(const Vertex& x0, const Vertex& x1);

    void VisualizePath(std::deque<Vertex>& path);

    Vector2i WorldToGrid(const Vector2f& world);
    Vector2f GridToWorld(const Vector2i& grid);

private:    
    VertexGrid _vertex_grid;
    std::deque<Vertex> _vertices;
    Vector2f _map_min, _map_max;
    Vector2f _goal;

    float _grid_size = 1.f;

    const vector_map::VectorMap& _map;
    f1tenth_course::VisualizationMsg& _msg;
};
} // namespace planning

#endif // RRT_H
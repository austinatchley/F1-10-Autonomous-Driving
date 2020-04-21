// #include <string>
#include <deque>

#include "f1tenth_course/VisualizationMsg.h"
#include "Vertex.h"
#include "vector_map/vector_map.h"

using std::string;

#ifndef RRT_H
#define RRT_H

namespace planning { 
using namespace Eigen;

class RRT {
public:
    RRT(const vector_map::VectorMap& map, f1tenth_course::VisualizationMsg& msg) : _map(map), _msg(msg) {}

    void Initialize();

    // Finds a path between cur and goal using RRT*
    void FindPath(const Vector2f& cur, const Vector2f& goal, std::deque<Vertex>& path);

    // Finds a path between cur and goal using basic RRT
    void FindNaivePath(const Vector2f& cur, const Vector2f& goal, std::deque<Vertex>& path);

    // Returns true if we are within reasonable distance of the provided goal
    bool ReachedGoal(const Vertex& pos, const Vertex& goal);

    // Returns true if the line connecting the two given points does not collide with any objects
    bool ObstacleFree(const Vertex& x0, const Vertex& x1);

    // Returns a point between x0 and x1
    Vertex Steer(const Vertex& x0, const Vertex& x1);

    Vertex& Nearest(const Vertex& x, std::deque<Vertex>& vertices);

    // Returns a vector of points to the vertices representing the neighbors of a given point
    void GetNeighbors(std::deque<Vertex>& vertices, const Vertex& x, std::vector<Vertex*>& neighbors);

    // Returns the result of the edge cost function (i.e. Euclidean distance between points)
    float Cost(const Vertex& x0, const Vertex& x1);

    void VisualizePath(std::deque<Vertex>& path);

private:
    Vector2f _map_min, _map_max;
    Vector2f _goal;

    const vector_map::VectorMap& _map;
    f1tenth_course::VisualizationMsg& _msg;
};
} // namespace planning

#endif // RRT_H
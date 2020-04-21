// #include <string>

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
    RRT(const vector_map::VectorMap& map) : _map(map) {}

    void Initialize();

    // Finds a path between cur and goal
    void FindPath(const Vector2f& cur, const Vector2f& goal, std::vector<Vertex>& path);

    // Returns true if we are within reasonable distance of the provided goal
    bool ReachedGoal(const Vertex& pos, const Vertex& goal);

    // Returns true if the line connecting the two given points does not collide with any objects
    bool ObstacleFree(const Vertex& x1, const Vertex& x2);

    // Returns a point between x1 and x2
    Vertex Steer(const Vertex& x1, const Vertex& x2);

    // Returns a vector of indices in the vertices vector representing the neighbors of a give point
    // std::vector<int> GetNeighbors(const Pose& x1);

    // Returns the result of the edge cost function (i.e. Euclidean distance between points)
    // float Cost(const Pose& x1, const Pose& x2);

    static void VisualizePlan(std::vector<Vertex>& plan, f1tenth_course::VisualizationMsg msg) {}

private:
    Vector2f _map_min, _map_max;
    Vector2f _goal;

    const vector_map::VectorMap& _map;
};
} // namespace planning

#endif // RRT_H
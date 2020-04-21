// #include <string>

#include "f1tenth_course/VisualizationMsg.h"
#include "Vertex.h"
#include "Pose.h"
#include "vector_map/vector_map.h"

using std::string;

#ifndef RRT_H
#define RRT_H

namespace planning {
class RRT {
public:
    explicit RRT(const string& map_file);

    // TODO: Should plan be a vector<Vertex> or a vector<Vector2f>? How should navigation interface with RRT?
    void MakePlan(const Pose& cur, const Pose& goal, std::vector<Vertex>& plan);

    // Returns true if we are within reasonable distance of the provided goal
    bool ReachedGoal(const Pose& pos, const Pose& goal);

    // Returns true if the line connecting the two given points does not collide with any objects
    bool ObjectFree(const Pose& x1, const Pose& x2);

    // Returns a point between x1 and x2
    // TODO: This might want to be indices in the vertices vector instead
    Pose& MoveTowardPoint(const Pose& x1, const Pose& x2);

    // Returns a vector of indices in the vertices vector representing the neighbors of a give point
    std::vector<int> GetNeighbors(const Pose& x1);

    // Returns the result of the edge cost function (i.e. Euclidean distance between points)
    float Cost(const Pose& x1, const Pose& x2);

    static void VisualizePlan(std::vector<Vertex>& plan, f1tenth_course::VisualizationMsg msg) {}

private:
    std::vector<Vertex> _vertices;

    Pose _goal;

    vector_map::VectorMap _map;
};
} // namespace planning

#endif // RRT_H
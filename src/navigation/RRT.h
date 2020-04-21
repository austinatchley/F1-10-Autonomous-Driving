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

    bool ReachedGoal(const Pose& pos, const Pose& goal);

    static void VisualizePlan(std::vector<Vertex>& plan, f1tenth_course::VisualizationMsg msg) {}

private:
    std::vector<Vertex> _vertices;

    Pose _goal;

    vector_map::VectorMap _map;
};
} // namespace planning

#endif // RRT_H
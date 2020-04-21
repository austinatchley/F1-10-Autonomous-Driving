#include "RRT.h"

using std::string;
using Eigen::Vector2f;

namespace planning {
RRT::RRT(const string& map_file) {
    _map.Load(map_file);
}

void RRT::MakePlan(const Pose& cur, const Pose& goal, std::vector<Vertex>& plan) {
    _goal = goal;

    plan.clear();
    plan.push_back(Vertex(_goal));
}

bool RRT::ReachedGoal(const Pose& pos, const Pose& goal) {
    return false;
}
} // namespace planning
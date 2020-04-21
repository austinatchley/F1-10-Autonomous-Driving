#include "RRT.h"

using std::string;
using Eigen::Vector2f;

namespace planning {
RRT::RRT(const string& map_file) {
    _map.Load(map_file);
}

void RRT::MakePlan(const Pose& cur, const Pose& goal, std::vector<GridVertex>& plan) {
    _goal = goal.pos;

    plan.clear();
    plan.push_back(GridVertex(_goal));
}

bool RRT::ReachedGoal(const Pose& pos, const Pose& goal) {
    return false;
}
} // namespace planning
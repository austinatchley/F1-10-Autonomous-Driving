#include "eigen3/Eigen/Dense"

using Eigen::Vector2f;

#ifndef POSE_H
#define POSE_H

namespace planning {
struct Pose {
    Pose(Vector2f pos, float angle) : pos(pos), angle(angle) {}

    Vector2f pos;
    float angle;
};

} // namespace planning

#endif // POSE_H
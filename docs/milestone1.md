# Checkpoint 1.3 Progress Report

Logan Zartman

Austin Atchley

## 1. Mathematical computation:
## 1-D time optimal controller:
1. Time integration
    1. Compute straight-line distance traveled since last time step
    2. Compute velocity as distance / timestep size
    3. Update distance (total traveled) using explicit Euler integration
2. State prediction in 3 timeframes: last sensor read (*S*), predicted at current time (*C*), predicted at motor actuation (*A*)
    1. Let speed\_S = norm(velocity)
    2. Let position\_S = distance
    3. Compute speed\_C and position\_C using kinematics with last acceleration output and estimated sensor→control latency
    4. Compute speed\_A and position\_A using kinematics with estimated control→actuation latency
    5. Estimate stopping position using estimated state at actuation
3. Motion
    1. Check estimated stopping position against target position
    2. If stopping position >= target position then
        1. Compute required deceleration to stop at target
        2. Apply deceleration
    3. Else accelerate at max acceleration

### LIDAR to point cloud computation:
for each point in point cloud:
    theta_i = theta_0 + i * theta_increment
    p_i = {r_i * cos(theta_i), r_i * sin(theta_i)}

### Obstacle detection computation:
1. Given command line flags for max distance and curvature
2. Assuming we already have the point cloud constructed
3. Free path length = max distance
4. Iterating through all points in point cloud:
    1. If point is not an obstacle (using formula presented in slides), skip.
    2. Find minimum of free path length with this point, and the free path length so far
5. Add free path length to our current position to get target position
6. Run 1-D TOC with our current position and this target position

### Obstacle avoidance reward function:

We calculate the obstacle avoidance reward for each path with the following:

```
R(c) = free_path_length(c) + (free_path_length^2 * clearance(c) * clearance_weight) + (distance_to_target * distance_weight)
```

### Obstacle avoidance integration:

We optimize the function on the interval of possible curvatures at each timestep by performing a naive search with a set number of discrete steps. After choosing the best curvature, we perform a [Golden Section Search](https://en.wikipedia.org/wiki/Golden-section_search) on the interval surrounding the best curvature.


## 2. Code organization:


## 3. Parameter tuning:


## 4. Challenges faced:


## 5. Contributions of each team member:

Austin - 

Logan - 

All of the code was pair-programmed with Logan primarily committing.

## 6. GitHub Repo
[https://github.com/austinatchley/F1-10-Autonomous-Driving](https://github.com/austinatchley/F1-10-Autonomous-Driving)

## 7. Video Demos

[]()

## 8. Future improvements:



# Checkpoint 1.3 Progress Report

Logan Zartman

Austin Atchley

## 1. Mathematical computation:

### LIDAR to point cloud computation:
We used the calculations presented in class:

for each point:

    theta_i = theta_0 + i * theta_increment
    p_i = {r_i * cos(theta_i), r_i * sin(theta_i)}

### Obstacle determination computation:
Get Point Cloud
Get desired curvature to travel along
free path length = [some large number, e.g. the range of the sensor]
Iterating through all points in point cloud:
If point is not an obstacle, skip.
Find minimum of free path length with this point, and the free path length so far
Use free path length as distance left to traverse
Run 1-D TOC with this free path length as distance left.

## 2. Code organization:
We refactored our 1D TOC to take only distance and curvature as parameters. Broadly, the functionality we added for this checkpoint determines the distance and curvature to feed 1D TOC.

We have a function called `_is_in_path` that returns a boolean value. It performs the calculations outlined in section 1.2. If the point is in our path, we compute the distance along our path to the point with `_get_dist_to_point`.

Also, if the curvature is close to 0, our code executes a bit differently. ...

## 3. Parameter tuning:
Length of car

Width of car

## 4. Challenges faced:
Visualization

## 5. Contributions of each team member:

Austin - Point cloud reconstruction and visualization. Refactoring and structure of obstacle detection.

Logan - Mathematical computations. Fine-tuning of obstacle detection.

All object detection code was pair-programmed with Austin committing, but both of us shared the load about equally.

## 6. GitHub Repo
[https://github.com/austinatchley/F1-10-Autonomous-Driving](https://github.com/austinatchley/F1-10-Autonomous-Driving)

## 7. Video Demos

### 2 meters distance on pre-measured tape with 1m/s max velocity and 3m/s^2 max acceleration
[https://drive.google.com/file/d/1-6I8ubXC_vUiwEjwKUGETv0ijHdRV1WZ/view?usp=sharing](https://drive.google.com/file/d/1-6I8ubXC_vUiwEjwKUGETv0ijHdRV1WZ/view?usp=sharing)

## 8. Future improvements:
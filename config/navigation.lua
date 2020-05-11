-- RRT/RRT* pathfinding parameters --
rrt_max_iter_frame = 1000 -- maximum iterations per frame
rrt_min_total_iter = 10000 -- minimum total iterations before stopping
rrt_max_iter = 50000 -- total max iterations before giving up
rrt_goal_tolerance = 0.5 -- meters (set less than 2*wall_dilation for safety)
rrt_wall_dilation = 0.15 -- meters
rrt_neighborhood_radius = 1.5 -- meters
rrt_neighbor_count = 20 -- number of neighbors for nearest neighbors
rrt_steering_eta = 1.0 -- meters

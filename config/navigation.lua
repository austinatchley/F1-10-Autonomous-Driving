-- RRT/RRT* pathfinding parameters --
rrt_max_iter_frame = 3000 -- maximum iterations per frame
rrt_extra_iter = 6000 -- minimum total iterations before stopping
rrt_max_iter = 60000 -- total max iterations before giving up
rrt_goal_tolerance = 1.75 -- meters
rrt_wall_dilation = 0.2 -- meters
rrt_steering_eta = 1.0 -- meters

rrt_neighborhood_radius = 1.5 -- meters
rrt_neighbor_count = 5 -- number of neighbors for nearest neighbors

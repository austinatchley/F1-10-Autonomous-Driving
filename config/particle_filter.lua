map = "maps/GDC1.txt"
init_x = 14.7
init_y = 14.24
init_r = 0

flag_location_smoothing = false
flag_laser_smoothing = false
flag_variance_thresh = true
flag_dist_update = true

-- particle distribution params
k1 = 0.40 -- tangent translation error from translation 
k2 = 0.05 -- tangent translation error from rotation
k3 = 0.01 -- normal translation error from translation
k4 = 0.15 -- normal translation error from rotation
k5 = 0.15 -- rotation error from translation
k6 = 0.30 -- rotation error from rotation

-- params for fixing incorrect odom calibration
k_trans_scale = 1.00 -- tangent translation scale
k_rot_scale   = 1.20 -- tangent rotation scale

-- observation likelihood params
correlation = 0.9
sigma = 0.25
if flag_variance_thresh then
    -- sigma = 1.85
end
d_long = .25
d_short = .2
s_max = 9.0
s_min = 0.1
stride = 10
update_dist = 0.05

-- resample params
resample_rate = 1
if flag_variance_thresh then
    -- resample_rate = 20
end
var_threshold = .5

location_smoothing = 0.75

-- RRT/RRT* pathfinding parameters --
rrt_max_iter_frame = 5000 -- maximum iterations per frame
rrt_min_total_iter = 40000 -- minimum total iterations before stopping
rrt_max_iter = 60000 -- total max iterations before giving up
rrt_goal_tolerance = 0.5 -- meters (set less than 2*wall_dilation for safety)
rrt_wall_dilation = 0.15 -- meters
rrt_neighborhood_radius = 1.5 -- meters
rrt_neighbor_count = 20
rrt_steering_eta = 1.0 -- meters

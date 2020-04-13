map = "maps/GDC1.txt"
init_x = 14.7
init_y = 14.24
init_r = 0

extra_features = false

flag_location_smoothing = extra_features
flag_laser_smoothing = extra_features
flag_variance_thresh = extra_features

-- particle distribution params
k1 = 0.70 -- tangent translation error from translation 
k2 = 0.05 -- tangent translation error from rotation
k3 = 0.01 -- normal translation error from translation
k4 = 0.05 -- normal translation error from rotation
k5 = 0.30 -- rotation error from translation
k6 = 0.70 -- rotation error from rotation

-- params for fixing incorrect odom calibration
k_trans_scale = 1.00 -- tangent translation scale
k_rot_scale   = 1.20 -- tangent rotation scale

-- observation likelihood params
correlation = 0.9
sigma = 2
if flag_variance_thresh then
    sigma = 1.5
end
d_long = .75
d_short = .75
s_max_offset = 1.0
s_min_offset = 0.05
stride = 10

-- resample params
resample_rate = 1
if flag_variance_thresh then
    resample_rate = 20
end
var_threshold = .025

location_smoothing = 0.75

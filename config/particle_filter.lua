map = "maps/GDC1.txt"
init_x = 14.7
init_y = 14.24
init_r = 0

k1 = 0.30 -- tangent translation error from translation 
k2 = 0.05 -- tangent translation error from rotation
k3 = 0.01 -- normal translation error from translation
k4 = 0.05 -- normal translation error from rotation
k5 = 0.05 -- rotation error from translation
k6 = 0.1  -- rotation error from rotation

correlation = 0.9
sigma = 4
d_long = .75
d_short = .75
s_max_offset = 1.0
s_min_offset = 0.05
stride = 10
import numpy as np

a_wall = np.array([[0,15],[10,15]])#each wall as [x,y],[x,y] of its ends
b_wall = np.array([[10,15],[10,25]])
c_wall = np.array([[15,15],[15,25]])
d_wall = np.array([[15,15],[25,15]])
e_wall = np.array([[15,10],[25,10]])
f_wall = np.array([[15,10],[15,0]])
g_wall = np.array([[10,0],[10,10]])
h_wall = np.array([[0,10],[10,10]])
wall_points = [a_wall, b_wall, c_wall, d_wall, e_wall, f_wall, g_wall, h_wall]
h_centering_thresh_min = 5#change target to centre if y coordinate below this
h_centering_thresh_max = 15 # y coordinate where h-travelling peds change target
h_centering_target =12.5

v_centering_thresh_max = 14.5#for travel axis 1
v_centering_thresh_min = 5 #change target to centre if x coordinate below this
v_centering_target = 15

dir_change_thresh = 17 #point in wron axis where ped changes direction 
import numpy as np
dx = 0.00001 

def grad_func (a_pos, argb, func): #find grad of some func at a_pos using finite difference
        a_pos_diffx = a_pos.copy()             # arg_b s whatever other argument function takes
        a_pos_diffy = a_pos.copy()
        a_pos_diffx[0] = a_pos[0] + dx         #finite differenced coordinates
        a_pos_diffy[1] = a_pos[1] + dx
        drdx = (func(a_pos_diffx, argb) - func(a_pos,argb))/dx # finite difference evaluation
        drdy = (func(a_pos_diffy, argb) - func(a_pos,argb))/dx
        grad_vector = [drdx, drdy] #grad vector
        return grad_vector

def closest_wall_point (point, wall):#geometry to find what point on a wall is closest to ped 
        relative_vec = point - wall[0] 
        wall_vec = wall[1]-wall[0]      
        scaled_point_vec = relative_vec/np.linalg.norm(wall_vec) 
        wallunitvec = wall_vec/np.linalg.norm(wall_vec) 
        scale_fac = np.dot(wallunitvec, scaled_point_vec)
        if scale_fac < 0.0: # if scale fac not between 0 and 1 is closest to ends of wall
            scale_fac = 0.0
        elif scale_fac > 1.0:
            scale_fac = 1.0
        point_wall_d = scale_fac*wall_vec - point + wall[0] #location of nearest wall point

        return point_wall_d
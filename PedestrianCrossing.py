import numpy as np
import random
import math
from matplotlib import pyplot
import matplotlib.animation as animation
from geometry import *
from model_constants import *
from utilities import grad_func, closest_wall_point
from postprocess import plot_data


input_flux = float(input("Pedestrian Flux (default 1 ped/s):") or 1)
input_sim_time = float(input("Simulation duration (default 30 s):") or 30)
ped_count = 0 # cumulative number of peds through domain
sim_duration = input_sim_time        #duration of sim
nt = int(sim_duration/dt) #number of timesteps
average_flux = input_flux *2     #total flux for both inlets
entry_chance = average_flux * dt #chance of a ped entering in a timestep
fast_deccel_allowed = True  #if true allow peds to deccelerate faster than accelerate
centre_lost_peds = True #if true stuck peds will change target to get back on their desired axis
dir_change_allowed = True #if true allow peds who get stuck far in wrong stream to change axis
downstream_weighted = True #if true peds downstream will have greater repulsive force than upstream

class Pedestrian:
    def __init__(self):
        self.speed_desired = random.uniform(1.1, 1.3) # scalar random desired speed between 1.1 and 1.3 for each ped
        start_coordinate = random.uniform(10.5,14.5) # random entry location 
        self.acceleration = []  # acceleratioon vector; calc at start of each TS
        self.closest_wall = []   #distance vector to nearest wall; calc at start of each TS
        self.travel_axis = random.randint(0,1) # travel directoion; 0 is west-east. 1 is south-north; both entrances have equal probability
        if self.travel_axis == 0:    # give start location and velocity based on axis of travel
            self.position = np.array([0,start_coordinate])       
            self.velocity = np.array([self.speed_desired,0]) #all peds enter at desired velocity along preferred axis
        else:
            self.position = np.array([start_coordinate,0])       
            self.velocity = np.array([0,self.speed_desired]) 

    def near_wall_repulse_func (self, point, walls): #find direction vector of nearest wall to point
        shortest_dist_vec = [domain_height, domain_length] # upper bound of wall distance for easy comparing
        for ws in range(len(walls)):    #check distance to every wall to find shortest
            wall = walls[ws]                   
            point_wall_dist = closest_wall_point(point,wall) #find distance to closest point on current wall
            if np.linalg.norm(point_wall_dist)<(np.linalg.norm(shortest_dist_vec)): # check if closest wall 
                shortest_dist_vec = point_wall_dist
        ped_wall_dist = (np.linalg.norm(shortest_dist_vec) - ped_radius) #scalar distance from ped surface to wall
        wall_repulse_func = -wall_const*math.exp(-ped_wall_dist*wall_exponent) #change sqrt to a better function
        self.closest_wall = shortest_dist_vec    #vector distance to nearest wall
        return wall_repulse_func

    def ab_repulsion( self, ap,bp): #repulsion function between ped A and ped B; grad of this for true repulsion
        ab_distance = np.linalg.norm(bp - ap)-2* ped_radius #distance between outer surface of peds
        ab_repulse_func = -interped_const *math.exp(-ab_distance*interped_exp)
        return ab_repulse_func

    def calc_interped_forces(self): #repulsion between ped and all others
        a_b_repulsion = []
        for other_ped_index in range(len(list_pedestrians)):#find position of all other peds
            if (other_ped_index!=pedindex):
                b_position = list_pedestrians[other_ped_index].position   #postion of current other ped
                grad_repulsion = grad_func(self.position, b_position, self.ab_repulsion) #repulsion from current other ped
                if downstream_weighted:         #increase repulsion of downstream ped if weighting on
                    if (b_position[self.travel_axis] - self.position[self.travel_axis]>0):
                        [gr * ds_weight_fac for gr in grad_repulsion]
                a_b_repulsion.append(grad_repulsion)
        self.totalpedrepulsion = np.sum(a_b_repulsion, axis=0)  #sum of repulsions from all other peds
        
    def calc_wall_forces (self): #only repulsion from nearest wall
            wall_force = grad_func(self.position, wall_points,self.near_wall_repulse_func)
            self.totalwallrepulsion = wall_force

    def advance(self):
        if centre_lost_peds: #if centre fix is on, lost peds change target to get back in their corridor
            if (self.travel_axis==0 and (self.position[1]>h_centering_thresh_max or self.position[1]<h_centering_thresh_min)):
                target = ([h_centering_target, domain_height/2])
            elif (self.travel_axis == 1 and (self.position[0]>v_centering_thresh_max or self.position[0]<v_centering_thresh_min)):   #if centre fix is on, lost peds target centre
                target = ([domain_length/2, v_centering_target])
        if self.travel_axis==0:   #travel towards distant point along corridor centreline
            target = ([domain_length*2, domain_height/2])  
        if self.travel_axis==1:
            target = ([domain_length/2,domain_height*2])
        target_vec = (target - self.position)/np.linalg.norm(target - self.position) #unit vector from current location to target 
        force_acceleration = mass*(self.speed_desired*target_vec -self.velocity) +self.totalpedrepulsion + self.totalwallrepulsion #calc acceleration based on forces at beginning of TS
        if (np.linalg.norm(force_acceleration) > max_acceleration): #reduce acceleration to max but keep its direction
            self.acceleration = force_acceleration
            for ac in range(len(self.acceleration)):
                if ((np.sign(self.acceleration[ac]) ==np.sign(self.velocity[ac])) or not fast_deccel_allowed):  #if fast deccel is allowed deccel is not capped
                    self.acceleration[ac] = (2*self.acceleration[ac]/np.linalg.norm(force_acceleration))
        else:
            self.acceleration = force_acceleration
        velocity_n = self.velocity + self.acceleration * dt #new  velocity based on start of TS forces
        position_n = self.position + velocity_n *dt
        self.position = position_n
        self.velocity = velocity_n
        if dir_change_allowed:      #help unstrick peds who stray too far into wrong corridor
            if (self.position[1]>dir_change_thresh and self.travel_axis==0):
                self.travel_axis = 1
            if (self.position[0]>dir_change_thresh and self.travel_axis==1):
                self.travel_axis = 0
       
        return self.position 

#advance through time -
position_array = [] # list of [x,y] coordinates of peds for all timesteps 
travel_axis_array = []  #matching list of what axis they belong to
list_pedestrians=[] #list of all peds currently extant
for ts in range(nt):        #stepping through time
    print('Time = ' + str(round((ts* dt),2)))
    print('Current number of peds:',len(list_pedestrians))
    position_array.append([])
    travel_axis_array.append([])
    currentpositions =  position_array[ts-1]
    entry_roll = random.random()         #chance of a pedestrian being introduced, axis randomised in init
    if (entry_chance>entry_roll):
        list_pedestrians.append(Pedestrian())
        ped_count = ped_count + 1
        print('Cumulative number of pedestrians:' , ped_count)
    if len(list_pedestrians):    #only do calcs when peds are present
        for pedindex in range(len(list_pedestrians)):
           list_pedestrians[pedindex].calc_interped_forces()
           list_pedestrians[pedindex].calc_wall_forces()
        for pedindex in range(len(list_pedestrians)):
           list_pedestrians[pedindex].advance()
           position_array[ts].append(list_pedestrians[pedindex].position)  #list of ts with all coordinates at each one
           travel_axis_array[ts].append(list_pedestrians[pedindex].travel_axis)  #list of ts with all coordinates at each one
    new_list_peds = [item for item in list_pedestrians if ((0<item.position[0] <25) and (0<item.position[1]<25))] # remove peds outside of domain
    list_pedestrians = new_list_peds

plot_data(position_array,travel_axis_array,wall_points, input_flux, nt)




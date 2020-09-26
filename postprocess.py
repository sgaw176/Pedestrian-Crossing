from matplotlib import pyplot
import matplotlib.animation as animation
import numpy as np
import math
from model_constants import *
from model_constants import domain_height, domain_length,dt
figdimension = 6
xpoints = []
ypoints = []

def plot_data(positions_at_time,axis_at_time,walls_loc, flux, no_timesteps):


#plotdata(position_array)
    for hm in range(len(positions_at_time)):# xy data for colormap
        tspoints = positions_at_time[hm]
        for hn in range(len(tspoints)):
           if tspoints[hn][0]:
               xpoints.append(tspoints[hn][0])
               ypoints.append(tspoints[hn][1])

    def setfigsize(figheight, figwidth, figax): #set figure axis so peds are at right scale
        leftmarg = figax.figure.subplotpars.left
        rightmarg = figax.figure.subplotpars.right
        topmarg = figax.figure.subplotpars.top
        botmarg = figax.figure.subplotpars.bottom
        figw = float(figwidth)/(rightmarg-leftmarg)
        figh = float(figheight)/(topmarg-botmarg)
        figax.figure.set_size_inches(figw, figh)

    def animate (pts):  #scatter plot over time
        ax2.cla()
        pyplot.xlim(0, 25)
        pyplot.ylim(0, 25)
        pyplot.text(2, 22, 'Φ=' +str(flux)+' pedestrians/s')
        pyplot.text(2, 21, 't=' +str(round((pts * dt),2)))

        for wps in walls_loc:#plot walls on animation
            pyplot.plot([wps[0][0],wps[1][0]],[wps[0][1],wps[1][1]], c ="k")
        fixedtimeposition = positions_at_time[pts]
        fixed_time_direction = axis_at_time[pts]
        for k in range(len(fixedtimeposition)):
            pedestriancoordinate = fixedtimeposition[k]
            ped_axis = fixed_time_direction[k]
            if ped_axis == 0:   #colour plot by direction of travel
                ped_color = 'r'
            else:
                ped_color = 'b'
            xplot = pedestriancoordinate[0]
            yplot = pedestriancoordinate[1]
            f2 = ax2.scatter(xplot,yplot, s = pedpointsize, facecolors='none',edgecolors=ped_color)

    f1 = pyplot.figure(1)
    ax1 = pyplot.gca()
    for wps in walls_loc:
            pyplot.plot([wps[0][0],wps[1][0]],[wps[0][1],wps[1][1]], c ="w")
    pyplot.xlim(0, 25)
    pyplot.ylim(0, 25)
    pyplot.text(2, 22, 'Φ=' +str(flux)+'pedestrians/s', color = 'white')
    f1 =ax1.hexbin(xpoints,ypoints,gridsize=(70,70),cmap='inferno')
    setfigsize(figdimension, figdimension,ax1)
    pedfigrad= ped_radius * figdimension/domain_height*72 #72 dpi
    pedpointsize = math.pi*pedfigrad**2
    f2 = pyplot.figure(2)
    ax2 = pyplot.gca()
    pyplot.xlim(0, 25)
    pyplot.ylim(0, 25)
    setfigsize(figdimension, figdimension,ax2)
    f_ani = animation.FuncAnimation(f2, animate, no_timesteps, interval =dt*2000) #fix display speed
    

    pyplot.show()

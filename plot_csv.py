
# 11/05/2022
# IN-LP
#
import numpy as np
import matplotlib.pyplot as plt
import signal
import os
signal.signal(signal.SIGINT, signal.SIG_DFL)


_, _, files = next(os.walk("./_csv_file"))
NN = len(files)

xx_csv = {}
Tlist = []

for ii in range(NN):
    xx_csv[ii] = np.genfromtxt("_csv_file/agent_{}.csv".format(ii), delimiter=',').T
    Tlist.append(xx_csv[ii].shape[1])

n_x = xx_csv[ii].shape[0]
print(n_x)
Tmax = min(Tlist)

xx = np.zeros((NN*n_x,Tmax))

for ii in range(NN):
    for jj in range(n_x):
        index_ii = ii*n_x+jj
        xx[index_ii,:] = xx_csv[ii][jj][:Tmax] # useful to remove last samples

plt.figure()
for x in xx:
    plt.plot(range(Tmax), x)  

block_var = False if n_x < 3 else True
plt.show(block=block_var)


if 1 and n_x == 2: # animation 
    plt.figure()
    dt = 4 # sub-sampling of the plot horizon
    #dt = 3
    for tt in range(0,Tmax,dt):
        xx_tt = xx[:,tt].T
        for ii in range(NN):
            index_ii =  ii*n_x + np.arange(n_x)
            xx_ii = xx_tt[index_ii]
            plt.plot(xx_ii[0],xx_ii[1], marker='o', markersize=15, fillstyle='none', color = 'tab:red')


        axes_lim = (np.min(xx)-1,np.max(xx)+1)
        #axes_lim = (-5,5)
        plt.xlim(axes_lim); plt.ylim(axes_lim)
        plt.plot(xx[0:n_x*NN:n_x,:].T,xx[1:n_x*NN:n_x,:].T, alpha=0.3) # uncomment to show trajectories

        plt.axis('equal')
        plt.text(0,3,"iter: {}".format(tt)) # write the iteration
        plt.text(0,2.5,"time: {}s".format(tt*5e-2)) # write the time (iteration*sampling time defined in the launch file)
        plt.grid()
        #plt.show(block=True)
        plt.show(block=False)
        #plt.pause(0.1)
        plt.pause(0.0001)
        if tt < Tmax - dt - 1:
            plt.clf()
    plt.show()

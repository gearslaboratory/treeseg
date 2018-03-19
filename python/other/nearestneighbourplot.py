#!/usr/bin/env python

import numpy as np
import math
import scipy.stats
import matplotlib.pyplot as plt

data = np.loadtxt('n.dat')

z_min = math.floor(min(data[:,2]))
z_max = math.ceil(max(data[:,2]))

step = 1

bins = np.arange(z_min,z_max,step)
idx = np.digitize(data[:,2],bins)

tmp = []
count = 1
final = []
for i in xrange(len(bins)):
	#this is stupid
	for j in xrange(len(data)):
		if(idx[j] == count):
			tmp.append(data[j][3])
	result = np.array(tmp)
	final.append(np.mean(result))
	count += 1
final = np.array(final)
print final

#plt.scatter(final,bins,color='k',label='Binned average')
#plt.plot([0.01052857,0.01052857],[-100,100],color='k',linestyle='--',label='Maximum')
#plt.gca().invert_xaxis()
#plt.xlim(0.012,0.004)
#plt.ylim(-5,20)
#plt.xlabel('Mean Euclidean distance from 6 nearest neighbours (m)')
#plt.ylabel('z(m)')
#plt.legend()
#plt.savefig('eucliddist',dpi=150)

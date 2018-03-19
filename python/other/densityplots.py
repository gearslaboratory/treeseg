#!/usr/bin/env python 

import sys
import numpy as np
import matplotlib.pyplot as plt

plt.figure()

complete = False
z = 0

for j in xrange(1,3):

	data = np.loadtxt(sys.argv[j])

	x_len = np.max(data[:,0]) - np.min(data[:,0]) 
	y_len = np.max(data[:,1]) - np.min(data[:,1])
	height = np.max(data[:,2]) - np.min(data[:,2])

	point_count = len(data)
	no_bins = int(height / 2)
	bin_width = height / (no_bins -1)

	hist,bbins = np.histogram(data[:,2],no_bins)

	#pyplot.figure()
	#pyplot.hist(hist,bbins)
	#pyplot.savefig('density.png',dpi=150)

#	print hist
#	print bbins

	tmp = []
	for i in xrange(len(hist)):
		tmp.append(hist[i] / (x_len * y_len * bin_width))
	density = np.array(tmp)

	spacing = np.delete(bbins,len(bbins)-1)

#	print density
#	print spacing

	if(j == 1):
		plt.barh(spacing,density,2,color='w',hatch='//',label='Original data')
	if(j == 2):
		plt.barh(spacing,density,2,color='k',alpha=0.33,label='Downsampled data') 

lg = plt.legend()
lg.draw_frame(False)
plt.xlabel('Density (p/m3)')
plt.ylabel('Z (m)')
plt.savefig('density.png',dpi=150)

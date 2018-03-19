#!/usr/bin/env python

import sys

from mpl_toolkits.basemap import Basemap
import matplotlib.pyplot as plt
import numpy

#m = Basemap(projection='mill',llcrnrlat=-60,urcrnrlat=90,llcrnrlon=-180,urcrnrlon=180,resolution='l')
#m.drawcoastlines(linewidth=0.1)
#m.drawcountries(linewidth=0.1)
#m.fillcontinents(color='grey',alpha=0.33)
#m.drawmapboundary(color='none')

latitude = numpy.loadtxt(sys.argv[1],dtype='S256')
longitude = numpy.loadtxt(sys.argv[2],dtype='S256')
vegetation = numpy.loadtxt(sys.argv[3],dtype='S256')

for i in xrange(len(latitude)):
	try:
		lat = float(latitude[i])
		lon = float(longitude[i])
		try:
			latp = float(latitude[i-1]
			lonp = float(longitude[i-1])
			if(lat == latp):
				count = count + 1
			else:
				xxx
		except:
			continue
	except:
		continue

#results = []
#count = 0
#for i in xrange(1,len(latitude)):
	



#if (latitude[i] == latitude[i-1]):
#		count = count + 1
#	else:
#		print latitude[i]
#
#	else:
#		steps = steps + 1	
#	else:
#		results.append([latitude[i-1],longitude[i-1],vegetation[i-1],count])
#		count = 1
#results = numpy.array(results)
#print len(results),results

#for i in xrange(len(latitude)):
#	if(latitude[i] != 'NA'):
#		x = float(latitude[i])
#		y = float(longitude[i])
#		m.scatter(x,y,3,marker='o',color='k')

#plt.savefig("test.pdf")



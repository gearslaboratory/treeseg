#!/usr/bin/env python

import sys
import numpy
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot

total_mass = numpy.loadtxt(sys.argv[1],dtype='S256')
aboveground_mass = numpy.loadtxt(sys.argv[2],dtype='S256')
belowground_mass = numpy.loadtxt(sys.argv[3],dtype='S256')
tb = []
agb = []
bgb = []
for i in xrange(len(total_mass)):
	if(total_mass[i] != 'NA' and aboveground_mass[i] != 'NA' and belowground_mass[i] !='NA'):
		tb.append(float(total_mass[i]))
		agb.append(float(aboveground_mass[i]))
		bgb.append(float(belowground_mass[i]))
tb = numpy.array(tb)
agb = numpy.array(agb)
bgb = numpy.array(bgb)

agbratio = agb/tb
bgbratio = bgb/tb


print len(tb)

print numpy.mean(agbratio),numpy.std(agbratio)
print numpy.mean(bgbratio),numpy.std(bgbratio)

fig_width_pt = 469.755
inches_per_pt = 1.0/72.27
#golden_mean = (numpy.sqrt(5.0)-1.0)/2.0
fig_width = fig_width_pt*inches_per_pt*1
fig_height = fig_width
#fig_height = fig_width*golden_mean
fig_size = [fig_width,fig_height]
params = {
		"backend": "pdf",
		"text.usetex": True,
		"font.family": "serif",
		"font.serif": ["Computer Modern"],
		"font.size": 11,
		"figure.figsize": fig_size
	}
matplotlib.rcParams.update(params)
matplotlib.pyplot.scatter(numpy.log(agb),numpy.log(bgb),color='k',alpha=0.5,s=0.5)
matplotlib.pyplot.xlabel('Total biomass ln(kg)')
matplotlib.pyplot.ylabel('Above-ground biomass ln(kg)')
matplotlib.pyplot.savefig('test.pdf')

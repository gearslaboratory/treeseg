#!/usr/bin/env python

import sys
import argparse
import numpy as np
import math
import random
import matplotlib.pyplot as plt

def Chave2014(d,h,wd):

	diameter = d * 100
	height = h
	wood_density =  wd / 1000
	biomass = 0.0559 * (diameter * diameter * height * wood_density)
	return biomass

def sortData(name):

	tmp = np.loadtxt(name,dtype='S16')
	data = np.zeros(len(tmp),dtype=[('country','S16'),('d',float),('h',float),('agb_obv',float),('wd',float),('agb_est',float)])
	for i in xrange(len(tmp)):
		for j in xrange(len(tmp[0])):
			data[i][j] = tmp[i][j]
		data[i]['agb_est'] = Chave2014(data[i]['d'],data[i]['h'],data[i]['wd'])
	return data

def allometricUncertainty(data,diameter,bins):

	if(len(bins) > 2):
		max_range = np.nan
		min_range = np.nan
		for i in xrange(1,len(bins)):
			if(diameter >= bins[i-1] and diameter <= bins[i]):
				min_range = bins[i-1]
				max_range = bins[i]
				break
	else:
		min_range = 0
		max_range = 100
	idx = []
	for j in xrange(len(data)):
		if(data[j]['d'] >= min_range and data[j]['d'] <= max_range):
			idx.append(j)
#	samples = random.randint(3,len(idx))			
#	idx = random.sample(idx,samples)
	residual_sum = 0
	for j in xrange(len(idx)):
		residual_sum += math.pow(np.log(data[idx[j]]['agb_obv'])-np.log(data[idx[j]]['agb_est']),2)
	stddev = math.sqrt(residual_sum/(len(idx)-1))
	correction_factor = math.exp(math.pow(stddev,2)/2) 
	allometricUncertainty = math.sqrt(math.pow(correction_factor,2)-1) 	
	return allometricUncertainty

data = sortData(sys.argv[1])

d = [0.1,0.4,0.75,1.35]
#d = np.linspace(0.1,1.5,4)

#runs = 100
#x = np.zeros((len(d)))
#y = np.zeros((len(d)))
#for i in xrange(len(d)):
#	tmp = np.zeros((runs))
#	for j in xrange(runs):
#		tmp[j] = allometricUncertainty(data,d[i],[0,0.1,0.4,0.75,10]) 
#	y[i] = np.mean(tmp)
#	x[i] = d[i]
#	#print x[i],y[i]

print allometricUncertainty(data,1,[0])

coefficients = np.polyfit(x,y,2)
polynomial = np.poly1d(coefficients)
xx = np.linspace(0.1,1.5,100)

x_s = [0.1,0.75,1.5]
y_s = np.zeros((len(x_s)))
for k in xrange(len(x_s)):
	y_s[k] = allometricUncertainty(data,x_s[k],[0])

fig, ax1 = plt.subplots()
ax2 = ax1.twinx()

ax1.scatter(data['d'],np.abs(data['agb_obv']-data['agb_est'])/data['agb_est'],color='k',marker='+',s=10)
ax1.set_xlim([0,np.max(data['d'])])
ax1.set_ylim([0,np.max(np.abs(data['agb_obv']-data['agb_est'])/data['agb_est'])])
ax1.set_xlabel('dbh (m)')
ax1.set_ylabel('Relative residual error')
ax2.set_ylabel('Regression uncertainty')

ax2.plot(xx,polynomial(xx),color='k')
ax2.plot(x_s,y_s,color='k',linestyle='--')
ax2.set_ylim([0,0.5])

plt.savefig('allomuncert.png',dpi=200)

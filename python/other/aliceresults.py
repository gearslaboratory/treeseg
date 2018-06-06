#!/usr/bin/env python

import sys
import numpy as np
import matplotlib.pyplot as plt
import numpy

data = numpy.loadtxt(sys.argv[1])

plt.scatter(data[:,0],data[:,1],color='r',label='Off')
plt.scatter(data[:,0],data[:,2],color='g',label='On')
plt.scatter(data[:,0],data[:,3],color='b',label='On(S)')

plt.plot([0,2000],[0,2000],color='k',label='1:1')

plt.xlim([0,2000])
plt.ylim([0,2000])

plt.legend()

plt.xlabel('Harvest AGB (kg)')
plt.ylabel('TLS AGB (kg)')

plt.savefig('alice_results.png',dpi=500)

#LEAF OFF
#plt.errorbar(reference_weight,off_volume*wood_density,yerr=off_uncert*wood_density,label='Leaf off',color='k',fmt='.',markersize=9,capsize=3,elinewidth=1)
#LEAF OFF + BARK
#plt.errorbar(reference_weight,(off_volume*(1-bark_fraction)*wood_density)+(off_volume*bark_fraction*bark_density),yerr=(off_uncert*(1-bark_fraction)*wood_density)+(off_uncert*bark_fraction*bark_density),label='Leaf off(wood+bark)',fmt='.',color='k',marker='*',markersize=9,capsize=3,elinewidth=1)
#LEAF ON
#plt.errorbar(reference_weight,(on_volume*(1-bark_fraction)*wood_density)+(on_volume*bark_fraction*bark_density),yerr=(on_uncert*(1-bark_fraction)*wood_density)+(on_uncert*bark_fraction*bark_density),label='Leaf on',fmt='.',color='k',marker='^',markersize=7,capsize=3,elinewidth=1)

#plt.plot(xline,yline,label='1:1',color='k')
#plt.xlabel('Reference AGB (kg)')
#plt.ylabel('QSM AGB (kg)')
#plt.axes().set_aspect('equal','box')
#plt.xlim([0,2000])
#plt.ylim([0,2000])
#plt.legend(loc=4)
#plt.show()
#plt.savefig('alice_results.png',dpi=150)


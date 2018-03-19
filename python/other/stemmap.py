#!/usr/bin/env python 

import sys
import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt(sys.argv[1])

plt.figure()
plt.scatter(data[:,0],data[:,1],data[:,2]*250,color='k',alpha=0.66)

plt.axes().set_aspect('equal','box')

plt.xlim([-30,30])
plt.ylim([-30,30])

plt.xlabel('x(m)')
plt.ylabel('y(m)')

plt.savefig('stem_map.png',dpi=150)

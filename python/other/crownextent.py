#!/usr/bin/env python 

import sys
import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt(sys.argv[1])

plt.figure()
plt.scatter(data[:,2],data[:,4],color='k')

plt.xlabel('DBH (m)')
plt.ylabel('Crown extent (m)')

plt.savefig('c_dbh.png',dpi=150)

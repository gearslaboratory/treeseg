#!/usr/bin/env python

import glob
import numpy as np

files = glob.glob('*.dat')
out = []
for i in xrange(len(files)):
	tmp = np.loadtxt(files[i])
	out.append([tmp[0][3],tmp[1][3],tmp[2][3]])
out = np.array(out)
np.savetxt('locations.xyz',out)	

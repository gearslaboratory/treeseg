#!/usr/bin/env python

import sys
import os
import numpy as np
	
for i in xrange(1,len(sys.argv)):
	tmp1 = sys.argv[i].split('/')
	tmp2 = tmp1[1].split('_')


	fname = 'on_'+tmp2[1]+'.pcd'


	f1 = open(sys.argv[i],'r')
	line = f1.readline()
	x = []
	y = []
	z = []
	while line:
		tmp = line.split()
		if(tmp[0] == 'v'):
			v1 = float(tmp[1])
			v2 = float(tmp[2])
			v3 = float(tmp[3])
			x.append(v1)
			y.append(v3)
			z.append(v2)
		line = f1.readline()	
	f1.close()
	run_string = 'xyzfilter on.pcd '+str(min(x))+' '+str(max(x))+' '+str(min(y))+' '+str(max(y))+' '+str(min(z))+' '+str(max(z))+' '+fname
	print run_string
	os.system(run_string)

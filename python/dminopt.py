#!/usr/bin/env python

import sys
import numpy

for x in xrange(1,len(sys.argv)):
	metadata = numpy.loadtxt(sys.argv[x])
	dmin_opt = numpy.nan
	v = numpy.nan
	min_cov = numpy.min(metadata[:,2,])
	max_conf = numpy.max(metadata[:,3])
	print min_cov *2, max_conf*0.95
	print "----"
	for m in xrange(len(metadata)):
		print metadata[m][2],metadata[m][3]
		if(metadata[m][2] < min_cov * 2 and metadata[m][3] > max_conf * 0.95):
			dmin_opt = metadata[m][0]
			v = metadata[m][1]
			break
	if(numpy.isnan(dmin_opt) == True):
		idx = numpy.argmin(metadata[:,2])
		dmin_opt = metadata[idx][0]	
		v = metadata[idx][1]
	print sys.argv[x],dmin_opt,v

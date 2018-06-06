#!/usr/bin/env python

import sys
import scipy.io
import numpy as np

def chave2014(dbh,h,wd):

	biomass = 0.0559 * dbh * dbh * h * wd
	return biomass

wd = 750
h = 44

data = scipy.io.loadmat(sys.argv[1])

tmp = []
length = 0
for i in xrange(len(data['CiB'][0][0])):
	if(length > 6):
		break
	else:
		dbh = data['Rad'][data['CiB'][0][0][i][0]-1][0] * 2 
		biomass = chave2014(dbh,h,wd)
		tmp.append([length,biomass])
		length += data['Len'][data['CiB'][0][0][i][0]-1][0]

data = np.array(tmp)
print data


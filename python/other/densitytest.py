#!/usr/bin/env python

import sys
import scipy.io
import numpy as np

data = scipy.io.loadmat(sys.argv[1])

trunk_volume = 0
trunk_bark_volume = 0

branch_volume = 0
branch_bark_volume = 0

volume = 0

for i in xrange(len(data['BOrd'])):
	for j in xrange(len(data['CiB'][i][0])):
		if(data['BOrd'][i][0] == 0):
			tmp_volume = data['Rad'][data['CiB'][i][0][j][0]-1][0]*data['Rad'][data['CiB'][i][0][j][0]-1][0]*data['Len'][data['CiB'][i][0][j][0]-1][0]*np.pi
			trunk_volume += tmp_volume * 0.95
			trunk_bark_volume += tmp_volume * 0.05
		else:
			tmp_volume = data['Rad'][data['CiB'][i][0][j][0]-1][0]*data['Rad'][data['CiB'][i][0][j][0]-1][0]*data['Len'][data['CiB'][i][0][j][0]-1][0]*np.pi
			branch_volume += tmp_volume * 0.95
			branch_bark_volume += tmp_volume * 0.05

print trunk_volume * 827
print trunk_bark_volume * 400


print branch_volume * 827
print branch_bark_volume * 400


	#	trunk_volume =
	#else:
	#	volume

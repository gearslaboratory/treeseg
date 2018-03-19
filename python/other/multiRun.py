#!/usr/bin/env python

import glob
import os

files = glob.glob("*.xyz")
for i in xrange(len(files)):
	tmp = files[i].split('.')
	results_file = 'results_'+tmp[0]+'.dat'
	nohup_file = 'nohup_'+tmp[0]+'.dat'
	run_string = 'runCylinderModel.py -i '+files[i]+' -m /Applications/MATLAB_R2013b.app/bin/matlab -o '+results_file
	os.system(run_string)

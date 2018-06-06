#!/usr/bin/env python

import glob
import os

files = glob.glob("*.obj")
for i in xrange(len(files)):
	tmp = files[i].split('_')
	outfile = tmp[0]+'_'+tmp[1]+'_'+tmp[2]+'.librat.bbox'
	plant_name = "'g plant "+tmp[1]+"'"
	run_string = 'obj2librat.py -i '+files[i]+' -b '+outfile+' -f 200 -n '+plant_name
	print(run_string)
	os.system(run_string)

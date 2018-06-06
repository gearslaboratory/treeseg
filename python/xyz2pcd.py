#!/usr/bin/env python

#Andrew Burt - a.burt.12@ucl.ac.uk

import sys
import os

for i in xrange(1,len(sys.argv)):
	f1 = open(sys.argv[i],'r')
	f2 = open('body.tmp','w')
	line = f1.readline()
	count = 0
	while line:
		tmp = line.split()
#		f2.write(tmp[0]+' '+tmp[1]+' '+tmp[2]+' '+tmp[3]+' '+tmp[4]+' '+tmp[5]+' '+tmp[6]+'\n')
		f2.write(tmp[0]+' '+tmp[1]+' '+tmp[2]+'\n')
		count += 1
		line = f1.readline()
	f3 = open('header.tmp','w')
	f3.write('VERSION .7\n')
	f3.write('FIELDS x y z\n')
	f3.write('SIZE 4 4 4\n')
	f3.write('TYPE F F F\n')
	f3.write('COUNT 1 1 1\n')
	f3.write('WIDTH '+str(count)+'\n')
	f3.write('HEIGHT 1\n')
	f3.write('VIEWPOINT 0 0 0 1 0 0 0\n')
	f3.write('POINTS '+str(count)+'\n')
	f3.write('DATA ascii\n')
	f1.close()
	f2.close()
	f3.close()
	tmp = sys.argv[i].split('.')
	name = tmp[0]+'.pcd'
	quick = 'cat header.tmp body.tmp > '+name
	os.system(quick)
	os.system('rm header.tmp body.tmp')

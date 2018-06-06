#!/usr/bin/env python

#Andrew Burt - a.burt.12@ucl.ac.uk

import sys

for i in xrange(1,len(sys.argv)):
	f1 = open(sys.argv[i],'r')
	tmp = sys.argv[i].split('.')
	outfile = tmp[0]+'.txt'
	f2 = open(outfile,'w')
	line = f1.readline()
	while line:
		tmp = line.split()
		try:
			float(tmp[0])
			f2.write(line)
			line = f1.readline()
		except ValueError:
			line = f1.readline()
	f1.close()		
	f2.close()

#!/usr/bin/env python

from sys import argv
import os

start=argv[1]
end=argv[2]

for c in range(int(start),int(end)+1):
	os.rename("cluster_%i.pcd" % c, "flcuster_%i.pcd" % c)
#!/usr/bin/env python

import sys
import numpy as np
import matplotlib.pyplot as plt

tmp = np.loadtxt(sys.argv[1],dtype='S16')
data = np.zeros(len(tmp),dtype=[('country','S16'),('d',float),('h',float),('agb_obv',float),('wd',float),('agb_est',float)])
for i in xrange(len(tmp)):
	for j in xrange(len(tmp[0])):
		data[i][j] = tmp[i][j]

plt.loglog(data['d']*data['d']*data['h']*data['wd'],data['agb_obv'],'o',color='k',ms=1)
plt.xlabel(r'$log(\rho D^2H)$')
plt.ylabel(r'$log(AGB)$')
plt.savefig('chave.png',dpi=150)

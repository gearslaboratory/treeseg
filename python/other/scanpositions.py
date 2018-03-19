#!/usr/bin/env python 

import numpy as np
import matplotlib.pyplot as plt

data = np.genfromtxt('gold0101.txt')

plt.figure()
plt.scatter(data[:,0],data[:,1],data[:,2]*250,color='k',alpha=0.1)


plt.axis('equal')


plt.scatter(-25,0,40,color='k',marker='*')
plt.scatter(0,25,40,color='k',marker='*')
plt.scatter(25,0,40,color='k',marker='*')
plt.scatter(0,-25,40,color='k',marker='*')
plt.scatter(0,0,40,color='k',marker='*')

plt.plot([-25,-25],[-25,25],color='k',linestyle='--')
plt.plot([-25,25],[25,25],color='k',linestyle='--')
plt.plot([25,25],[25,-25],color='k',linestyle='--')
plt.plot([25,-25],[-25,-25],color='k',linestyle='--')

#x = [0,20,40,60,80,100]
#y = [0,20,40,60,80,100]

#tmp = []

#for i in  xrange(len(x)):
#	for j in xrange(len(y)):
#		tmp.append([x[i],y[j]])

#total = np.array(tmp)

#plt.scatter(total[:,0],total[:,1],50,color='k',marker='*')
plt.axes().set_aspect('equal','box')


#plt.plot([-25,-50],[-25,50],color='k',linestyle='--')
#plt.plot([-25,50],[25,50],color='k',linestyle='--')
#plt.plot([25,50],[25,-50],color='k',linestyle='--')
#plt.plot([25,-50],[-25,-50],color='k',linestyle='--')


#plt.xlim([-30,30])
#plt.ylim([-60,60])
#plt.axis('scaling')

plt.xlabel('x(m)')
plt.ylabel('y(m)')

plt.savefig('plot.png',dpi=150)

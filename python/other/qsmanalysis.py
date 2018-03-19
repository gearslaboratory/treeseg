#!/usr/bin/env python 

import sys
import numpy as np
import matplotlib.pyplot as plt
import scipy.io
import scipy.interpolate

#data = np.loadtxt(sys.argv[1])

###DMIN VS VOLUME###

#plt.figure()
#plt.scatter(data[:,0],data[:,1],color='k')
#plt.xlabel(r'dmin ($m$)')
#plt.ylabel(r'Volume ($m^3$)')
#plt.savefig('dmin_vol.png',dpi=150)

###DMIN VS CV###

#plt.figure()
#plt.scatter(data[:,0],data[:,2],color='k')
#plt.xlabel(r'dmin ($m$)')
#plt.ylabel(r'Standard error of mean ($m^3$)')
#plt.savefig('dmin_cv.png',dpi=150)

###DMIN VS DBH FIT###

#plt.figure()
#plt.scatter(data[:,0],data[:,3],color='k')
#plt.xlabel(r'dmin ($m$)')
#plt.ylabel('Normalised mean cylinder to cloud DBH conformity')
#plt.savefig('dmin_dbh.png',dpi=150)

###DMIN VS EVERYTHIN###

#cv_coefficients = np.polyfit(data[:,0],data[:,2],4)
#cv_polynomial = np.poly1d(cv_coefficients)

#dbh_coefficients = np.polyfit(data[:,0],data[:,3],6)
#dbh_polynomial = np.poly1d(dbh_coefficients)

#xx = np.linspace(0,0.1,100)

#fig,ax1 = plt.subplots()
#ax2 = ax1.twinx()

#ax2.plot([0.03535354,0.03535354],[0,5],color='k',linestyle='--',label='Optimised dmin')
#ax1.plot(xx,cv_polynomial(xx),color='k')
#ax2.plot(xx,dbh_polynomial(xx),color='k',linestyle='-.')
#ax1.scatter(data[:,0],data[:,2],color='k',marker='+',s=40)
#ax2.scatter(data[:,0],data[:,3],color='k')
#ax1.set_xlim([0,0.1])
#ax1.set_ylim([0,0.18])
#ax2.set_ylim([0,1.25])
#ax1.set_xlabel(r'dmin ($m$)')
#ax1.set_ylabel(r'Standard error of mean ($m^3$)')
#ax2.set_ylabel('Normalised mean cylinder to cloud DBH conformity')
#plt.legend()
#plt.savefig('overall.png',dpi=150)

###DMIN OPTIM VARIATION###

#b = [3,5,10,15,20,30,50,100]
#c = [0.0113885,0.00601066,0.00473423,0.0031831,0.00248284,0.00214235,0.00183963,0.00183659]
#cv_coefficients = scipy.interpolate.interp1d(b,c)
#cv_polynomial = np.poly1d(cv_coefficients)
#cv_polynomial = scipy.interpolate.interp1d(b,c,kind='slinear')
#xx = np.linspace(3,100,1000)
#plt.plot(xx,cv_polynomial(xx),color='k',linestyle='-.')
#plt.scatter(b,c,color='k')
#plt.xlim([0,110])
#plt.ylim([0,0.015])
#plt.xlabel('Runs')
#plt.ylabel(r'Standard error of mean ($m^3$)')
#print (data[:,7]-data[:,5])/1.96
#plt.savefig('optimised_dmin.png',dpi=150)

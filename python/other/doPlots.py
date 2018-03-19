#!/usr/bin/env python

#Andrew Burt - a.burt.12@ucl.ac.uk

import sys
import numpy as np
import matplotlib.pyplot as plt

def loadData(name):

	data = np.load(name)
	lidar = data['arr_0'] 
	field = data['arr_1']
	matched = data['arr_2']
	return lidar,field,matched

def matched_agb(matched):


#	sum_b = 0
#	for i in xrange(len(matched)):
#		sum_b += np.abs(matched[i]['lb']-matched[i]['fb']) / matched[i]['lb']
#	print sum_b / len(matched)

	print sum(matched['lb']), sum(matched['lb_max']-matched['lb'])
	print sum(matched['fb']), sum(matched['fb_max']-matched['fb_min'])


	plt.figure()
	plt.plot([0,2500],[0,2500],color='k')
	plt.errorbar(matched['fb'],matched['lb'],xerr=np.abs(matched['fb_max']-matched['fb']),yerr=np.abs(matched['lb_max']-matched['lb']),fmt='.',color='k')
	#plt.plot([agb_min,agb_max],[agb_min,agb_max],color='k')
	plt.axes().set_aspect('equal','box')
	plt.xlabel('Allometry AGB (kg)')
	plt.ylabel('QSM AGB (kg)')
	plt.savefig('matched_agb.png',dpi=150)

def lidar_lidar_agb(lidar):

	sum_b = 0
	for i in xrange(len(lidar)):
		sum_b += np.abs(lidar[i]['lb']-lidar[i]['fb']) / lidar[i]['lb']
	print sum_b / len(lidar)

	print sum(lidar['lb']), sum(lidar['lb_max']-lidar['lb'])
	print sum(lidar['fb']), sum(lidar['fb_max']-lidar['fb_min'])


	agb_min = 0
	#agb_max = max(np.max(lidar['lb']),np.min(lidar['fb']))
	agb_max = 30000
	plt.figure()
	plt.errorbar(lidar['fb'],lidar['lb'],xerr=np.abs(lidar['fb_max']-lidar['fb']),yerr=np.abs(lidar['lb_max']-lidar['lb']),fmt='.',color='k')
	plt.plot([agb_min,agb_max],[agb_min,agb_max],color='k')
	plt.axes().set_aspect('equal','box')
	plt.xlim([0,30000])
	plt.ylim([0,30000])
	plt.xlabel('Allometry (DBH and H from TLS) AGB (kg)')
	plt.ylabel('QSM AGB (kg)')
	plt.savefig('lidar_lidar_agb.png',dpi=150)

def height_extent(lidar):

	###PLOT HEIGHT VS EXTENT###

	coefficients = np.polyfit(lidar['le']*lidar['lh'],lidar['lb'],3)
	polynomial = np.poly1d(coefficients)
	print coefficients
	xx = np.linspace(0,5000,100)
	f1 = open('eh.dat','w')
	for i in xrange(len(lidar)):
		tmp = str(lidar[i]['le']*lidar[i]['lh'])+' '+str(lidar[i]['lb'])+'\n'
		f1.write(tmp)
	f1.close()
	plt.figure()
	plt.scatter(lidar['le']*lidar['lh'],lidar['lb'],color='k',s=6)
	plt.plot(xx,polynomial(xx),color='k',linestyle='--')
	plt.grid()
	plt.xlim([0,np.max(lidar['le']*lidar['lh'])+500])
	plt.ylim([0,np.max(lidar['lb'])+500])
	plt.xlabel('eH (m^2)')
	plt.ylabel('TLS AGB (kg)')
	plt.savefig('eh.png',dpi=150)

def matched_d_h(matched):

	###PLOT MATCHED DIAMETERS AND HEIGHTS###

	plt.figure()
	plt.axes().set_aspect('equal','box')
	plt.xlim([0,0.6])
	plt.ylim([0,0.6])
	plt.plot([0,1],[0,1],color='k')
	plt.scatter(matched['fdbh'],matched['ldbh'],color='k')
	plt.xlabel('DBH from field inventory (m)')
	plt.ylabel('DBH from TLS (m)')
	plt.savefig('matched_diameter',dpi=150)

	plt.figure()
	plt.axes().set_aspect('equal','box')
	plt.xlim([10,30])
	plt.ylim([10,30])
	plt.plot([0,40],[0,40],color='k')
	plt.scatter(matched['fh'],matched['lh'],color='k')
	plt.xlabel('Height from field inventory (m)')
	plt.ylabel('Height from TLS (m)')
	plt.savefig('matched_height',dpi=150)

	sum_b = 0
	for i in xrange(len(matched)):
		sum_b += np.abs(matched[i]['lh']-matched[i]['fh']) / matched[i]['lh']
	print sum_b / len(matched)


if __name__ == "__main__":

	lidar,field,matched = loadData(sys.argv[1])
	#matched_agb(matched)
	lidar_lidar_agb(lidar)
	#height_extent(lidar)
	#matched_d_h(matched)

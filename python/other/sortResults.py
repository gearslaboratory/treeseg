#!/usr/bin/env python

#Andrew Burt - a.burt.12@ucl.ac.uk

import sys
import argparse
import numpy as np
from doAllometry import allometricAGB

class sortResults:

	def __init__(self,lidar_data,field_data,matched_data,allometry_data,results_string):

		lidar = self.readData(lidar_data)
		field = self.readData(field_data)
		matched = self.readData(matched_data)
		self.allom_data = allometry_data
		self.field = self.sortData(field,'f')
		self.lidar = self.sortData(lidar,'l')
		self.matched = self.sortData(matched,'m')
		np.savez(results_string,self.lidar,self.field,self.matched)
#		print "LIDAR DATA:"
#		print self.lidar
#		print "FIELD DATA:"
#		print self.field
#		print "MATCHED DATA:"
#		print self.matched
	
	def readData(self,name):

		if(name != False):
			data = np.loadtxt(name,dtype='S16')
			return data
		else:
			return False

	def sortData(self,data,type_d):

		#FIELD -> fid,fspecies,fx,fy,fh,fdbh,fpom,wd,fb,fb_mim,fb_max

		if(type_d == 'f'):
			if(data != False):
				out = np.zeros(len(data),dtype=[('fid','S16'),('fspecies','S16'),('fx',float),('fy',float),('fh',float),('fdbh',float),('fpom',float),('wd',float),('fb',float),('fb_min',float),('fb_max',float)])
				for i in xrange(len(data)):
					for j in xrange(len(data[0])):
						out[i][j] = data[i][j]
				#agb_total = 0
				#agb_total_square = 0
				for k in xrange(len(out)):
					out['fb'][k],out['fb_min'][k],out['fb_max'][k]=self.getAllometry(out['fdbh'][k],out['fh'][k],out['wd'][k])
				#	agb_total += out['fb'][k]
				#	agb_total_square += pow(out['fb'][k],2)
				#plot_uncertainty = 0.57 * np.sqrt(agb_total_square)/agb_total
				#print plot_uncertainty
				return out

		#MATCHED -> fid,lid,fspecies,fx,fy,fh,fdbh,fpom,wd,lx,ly,lh,ldbh,le,lv,lv_min,lv_max,fb,fb_min,fb_max,lb,lb_min,lb_max

		if(type_d == 'm'):
			if(data != False):
				out = np.zeros(len(data),dtype=[('fid','S16'),('lid','S16'),('fspecies','S16'),('fx',float),('fy',float),('fh',float),('fdbh',float),('fpom',float),('wd',float),('lx',float),('ly',float),('lh',float),('ldbh',float),('le',float),('lv',float),('lv_min',float),('lv_max',float),('fb',float),('fb_min',float),('fb_max',float),('lb',float),('lb_min',float),('lb_max',float)])
				for i in xrange(len(data)):
					for j in xrange(len(data[0])):
						out[i][j] = data[i][j]		
				for k in xrange(len(out)):
					out['fb'][k],out['fb_min'][k],out['fb_max'][k]=self.getAllometry(out['fdbh'][k],out['fh'][k],out['wd'][k])
					out['lb'][k] = out['lv'][k]*out['wd'][k]
					out['lb_min'][k] = out['lv_min'][k]*out['wd'][k]
					out['lb_max'][k] = out['lv_max'][k]*out['wd'][k]
				return out

		#LIDAR -> lid,lx,ly,lh,ldbh,le,lv,lv_min,lv_max,wd,lb,lb_min,lb_max,fb,fb_min,fb_max

		if(type_d == 'l'):
			if(data != False):	
				out = np.zeros(len(data),dtype=[('lid','S16'),('lx',float),('ly',float),('lh',float),('ldbh',float),('le',float),('lv',float),('lv_min',float),('lv_max',float),('wd',float),('fb',float),('fb_min',float),('fb_max',float),('lb',float),('lb_min',float),('lb_max',float)])
				for i in xrange(len(data)):
					for j in xrange(len(data[0])):
						out[i][j] = data[i][j]
				for k in xrange(len(out)):
					out['wd'][k] = self.getDensity()
					out['fb'][k],out['fb_min'][k],out['fb_max'][k]=self.getAllometry(out['ldbh'][k],out['lh'][k],out['wd'][k])
					out['lb'][k] = out['lv'][k]*out['wd'][k]
					out['lb_min'][k] = out['lv_min'][k]*out['wd'][k]
					out['lb_max'][k] = out['lv_max'][k]*out['wd'][k]
				return out

	def getAllometry(self,diameter,height,density):

		allom = allometricAGB()
		if(self.allom_data != False):
			biomass,uncertainty = allom.doAllometry(diameter,height,density,self.allom_data)
			upper = biomass+((biomass*uncertainty)/2)
			lower = biomass-((biomass*uncertainty)/2)
			return biomass,lower,upper
		else:
			biomass = allom.doAllometry(diameter,height,density)
			return biomass,np.nan,np.nan

	def getDensity(self):

		try:
			#density = np.mean(self.field['wd'])
			density = 745
		except Exception:
			#defaulting to 745 - Lope2 plot wd
			density = 745.0
		return density

if __name__ == "__main__":

	parser = argparse.ArgumentParser()
	parser.add_argument('-m','--matched_data',default=False,help='matched data')
	parser.add_argument('-l','--lidar_data',default=False,help='lidar data')
	parser.add_argument('-f','--field_data',default=False,help='field data')
	parser.add_argument('-a','--allometry_data',default=False,help='allometry data')
	parser.add_argument('-r','--results_string',default='tls',help='results string')
	args = parser.parse_args()
	example = sortResults(args.lidar_data,args.field_data,args.matched_data,args.allometry_data,args.results_string)

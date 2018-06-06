#!/usr/bin/env python

import sys
import argparse
import numpy as np
import scipy.stats
import math
import random

class allometricAGB:

	def Chave2014(self,d,h,wd):

		#d = cm
		#h = m
		#wd = g/cm^3 
		diameter = d * 100
		height = h
		wood_density =  wd / 1000
		#biomass = 0.0673*math.pow(diameter*diameter*height*wood_density,0.976)
		biomass = 0.0559 * (diameter * diameter * height * wood_density)
		return biomass

	def sortAllometryData(self,name):

		tmp = np.loadtxt(name,dtype='S16')
		data = np.zeros(len(tmp),dtype=[('country','S16'),('d',float),('h',float),('agb_obv',float),('wd',float),('agb_est',float)])
		for i in xrange(len(tmp)):
			for j in xrange(len(tmp[0])):
				data[i][j] = tmp[i][j]
			data[i]['agb_est'] = self.Chave2014(data[i]['d'],data[i]['h'],data[i]['wd'])
		return data

#	def getRegressionPlots(self):
#
#		import matplotlib.pyplot as plt
#		
#		min_dbh = np.min(self.allometry_data['d'])
#		max_dbh = np.max(self.allometry_data['d'])
#		min_h = np.min(self.allometry_data['h'])
#		max_h = np.max(self.allometry_data['h'])	
#		d_bins = [min_dbh,0.1,0.25,0.5,0.75,1,1.5,max_dbh]
#		h_bins = [min_h,5,10,15,20,30,40,50,max_h]
#		#
#		fig = plt.figure(1)
#		plot = fig.add_subplot(111)
#		plt.hist(self.allometry_data['d'],d_bins,rwidth=0.5,align='mid',color='k')
#		plt.xticks(d_bins)
#		plot.tick_params(axis='both', which='major', labelsize=4)
#		plot.tick_params(axis='both', which='minor', labelsize=4)
#		plt.xlabel('Binned DBH (m)',fontsize=4)
#		plt.ylabel('Frequency',fontsize=4)
#		plt.savefig('dbh_bins.png',dpi=150)
#		#
#		fig = plt.figure(2)
#		plot = fig.add_subplot(111)
#		plt.hist(self.allometry_data['h'],h_bins,rwidth=0.5,align='mid',color='k')
#		plt.xticks(h_bins)
#		plot.tick_params(axis='both', which='major', labelsize=4)
#		plot.tick_params(axis='both', which='minor', labelsize=4)
#		plt.xlabel('Binned height (m)',fontsize=4)
#		plt.ylabel('Frequency',fontsize=4)
#		plt.savefig('h_bins.png',dpi=150)

#	def measurementUncertainty(self,diameter_uncertainty,height_uncertainty,density_uncertainty,correlation_coefficient):
#
#		alpha = 2
#		bravo = 1
#		delta = 1
#		term1 = math.pow(alpha,2) * math.pow(diameter_uncertainty,2)
#		term2 = math.pow(bravo,2) * math.pow(height_uncertainty,2)
#		term3 = math.pow(delta,2) * math.pow(density_uncertainty,2)
#		term4 = 2 * alpha * bravo * (1-correlation_coefficient) * (term1 + term2)
#		measurement_uncertainty = math.sqrt(term1 + term2 + term3 + term4)
#		return measurement_uncertainty
		

	def allometricUncertainty(self,monteCarlo=False,dbh_classification=False):

		####NUMBER OF RUNS####
		NUMBER_OF_RUNS = 1000
		######################


	#	if(dbh_classification == True):
			
			


#		if(monteCarlo == False):
#			if(binning == False):
#				residual_sum = 0
#				for i in xrange(len(self.allometry_data)):
#					residual_sum += math.pow(np.log(self.allometry_data['agb_obv'][i])-np.log(self.allometry_data['agb_est'][i]),2)
#				stddev = math.sqrt(residual_sum/(len(self.allometry_data)-1))
#				correction_factor = math.exp(math.pow(stddev,2)/2) 
#				allometricUncertainty = math.sqrt(math.pow(correction_factor,2)-1) 
#				print allometricUncertainty
#
#
#
#		if(monteCarlo == True):
#			if(binning == False):
#				tmp = []
#				for i in xrange(NUMBER_OF_RUNS):	
#					samples = random.randint(2,len(self.allometry_data))
#					idx = random.sample(self.allometry_data,samples)
#					residual_sum = 0
#					for i in xrange(len(idx)):	
#						residual_sum += math.pow(np.log(idx[i][3])-np.log(idx[i][5]),2)
#					stddev = math.sqrt(residual_sum/(len(idx)-1))
#					correction_factor = math.exp(math.pow(stddev,2)/2) 
#					tmp.append(math.sqrt(math.pow(correction_factor,2)-1)) 
#				allometricUncertainty = np.mean(tmp)
#				std = np.std(tmp)
#				limits = scipy.stats.norm.interval(0.95,loc=allometricUncertainty,scale=std)
#				print allometricUncertainty,limits

		#Peturb regression line through resample of allometry data
		#Random resample within dbh classification
		#bins = nbins
		#d_min = np.min(self.regression_data['d'])
		#d_max = np.max(self.regression_data['d'])	
		#dbh_bins = np.logspace(np.log(d_min),np.log(d_max),bins,base=np.e)
		#dbh_bins[0] = 0
		#dbh_bins[bins-1] = 1000
		#range_min = np.nan
		#range_max = np.nan	
	#	if(len(bins) > 2):
	#		for i in xrange(1,len(bins)):
	#			if(self.diameter >= bins[i-1] and self.diameter <= bins[i]):
	#				range_min = bins[i-1]
	#				range_max = bins[i]
	#				break
	#	else:
	#		range_min = 0
	#		range_max = 10
		#idx = []
		#for i in xrange(len(self.regression_data)):
		#	if(self.regression_data[i]['d'] >= range_min and self.regression_data[i]['d'] <= range_max):
		#		idx.append(i)
		#if(monteCarlo == True):
		#	samples = random.randint(2,len(idx))			
		#	idx = random.sample(idx,samples)
		#residual_sum = 0
		#for j in xrange(len(idx)):
		#		residual_sum += math.pow(np.log(self.regression_data[idx[j]]['agb_obv'])-np.log(self.regression_data[idx[j]]['agb_est']),2)
		#stddev = math.sqrt(residual_sum/(len(idx)-1))
		#correction_factor = math.exp(math.pow(stddev,2)/2) 
		#allometricUncertainty = math.sqrt(math.pow(correction_factor,2)-1) 
		#return allometricUncertainty

	def doAllometry(self,diameter,height,density,doUncertainty=False,monteCarlo=False,dbh_classification=False):


		biomass = self.Chave2014(diameter,height,density)
		uncertainty = np.nan

		if(doUncertainty != False):
						

		return biomass,uncertainty			

		###
		#MEASUREMENT UNCERTAINTY
		###
		#diameter_uncertainty = 0.03
		#height_uncertainty = 0.125
		#density_uncertainty = 0.1
		#correlation_coefficient = 0.9
		#measurement = self.measurementUncertainty(diameter_uncertainty,height_uncertainty,density_uncertainty,correlation_coefficient)	
		###
		#ALLOMETRIC UNCERTAINTY
		###
		#if(monteCarlo == False):
	#		allometric = self.allometricUncertainty(bins)
	#	else:
	#		tmp = np.zeros((monteCarlo,1))
	#		for i in xrange(monteCarlo):
	#			tmp[i] = self.allometricUncertainty(bins,monteCarlo=True)
	#		allometric = np.mean(tmp)
		###
	#	uncertainty = measurement + allometric
		#print uncertainty
	#	return biomass,uncertainty

if __name__ == "__main__":

	parser = argparse.ArgumentParser()
	parser.add_argument('-d','--dbh',default=False,help='diameter at breast height')
	parser.add_argument('-l','--height',default=False,help='tree height')
	parser.add_argument('-w','--density',default=False,help='wood density')
	parser.add_argument('-a','--allometry_data',default=False,help='allometry data')
	parser.add_argument('-b','--dbh_classification',default=False,help='dbh_classification')
	parser.add_argument('-m','--montecarlo',default=False,help='run montecarlo')
	args = parser.parse_args()
	example = allometricAGB()
	biomass,uncertainty = example.doAllometry(float(args.dbh),float(args.height),float(args.density),doUncertainty=args.allometry_data,monteCarlo=args.montecarlo,dbh_classification=args.dbh_classification)
	print biomass,uncertainty
#	example.allometricUncertainty(monteCarlo=True,dbh_classification=False)
#	biomass,uncertainty = example.generateUncertainty(args.dbh_bins,args.montecarlo)
#	print biomass,uncertainty

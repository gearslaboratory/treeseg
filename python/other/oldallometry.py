#!/usr/bin/env python

#Andrew Burt - a.burt.12@ucl.ac.uk

import argparse
import numpy
import math
import random
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot

def chave2014(dbh,h,wd):
	
	dbh =  dbh * 100
	wd = wd / 1000
	#agb = 0.0673 * math.pow(wd*dbh*dbh*h,0.976)
	agb = 0.0559 * (wd * dbh * dbh * h)
	return agb	

def cienciala2008(dbh,h):

	p0 = -3.069
	p1 = 2.137
	p2 = 0.661
	ld = 0.999
	agb = ld * numpy.exp(p0 + p1 * numpy.log(dbh) + p2 * numpy.log(h))
	return agb 

def sortAllomData(fname):

	tmp = numpy.loadtxt(fname,dtype='S256')
	allomdata = numpy.zeros(len(tmp),dtype=[('country','S256'),('dbh',float),('h',float),('agb_obv',float),('wd',float),('agb_est',float)])
	for i in xrange(len(tmp)):
		for j in xrange(len(tmp[0])):
			allomdata[i][j] = tmp[i][j]
		allomdata[i]['agb_est'] = chave2014(allomdata[i]['dbh'],allomdata[i]['h'],allomdata[i]['wd'])
	return allomdata

def allomUncert(fname,montecarlo,diameterclass):

	allomdata = sortAllomData(fname)
	###
	DELTA_D = 0.15
	RUNS = 1000
	###
	dbh = numpy.linspace(0.1,1.5,100)
	###
	allometric_uncertainty = []
	for i in xrange(len(dbh)):
		###
		data = []
		for j in xrange(len(allomdata)):
			if(diameterclass == True):
				if(allomdata[j][1] > dbh[i]-DELTA_D and allomdata[j][1] < dbh[i]+DELTA_D):		
					data.append(allomdata[j])
			else:
				data.append(allomdata[j])
		###
		runs = 1
		if(montecarlo == True):
			runs = RUNS
		#######################
		#regression uncertainty
		#######################
		r_uncert_list = []
		for k in xrange(runs):
			idx = data
			if(montecarlo == True):
				samples = random.randint(3,len(data))
				idx = random.sample(data,samples)
			residual_sum = 0
			for l in xrange(len(idx)):
				residual_sum += math.pow(numpy.log(idx[l][3])-numpy.log(idx[l][5]),2)
			stddev = math.sqrt(residual_sum/(len(idx)-1))
			correction_factor = math.exp(math.pow(stddev,2)/2)
			r_uncert = math.sqrt(math.pow(correction_factor,2)-1)
			r_uncert_list.append(r_uncert)
		r_uncert_list = numpy.array(r_uncert_list)
		regression_uncertainty = numpy.mean(r_uncert_list)
		########################
		#measurement uncertainty
		########################
		diameter_uncertainty = 0.05
		height_uncertainty = 0.1
		density_uncertainty = 0.1
		correlation_coefficient = 0.8
		###
		alpha = 2
		bravo = 1
		delta = 1
		term1 = math.pow(alpha,2) * math.pow(diameter_uncertainty,2)
		term2 = math.pow(bravo,2) * math.pow(height_uncertainty,2)
		term3 = math.pow(delta,2) * math.pow(density_uncertainty,2)
		term4 = 2 * alpha * bravo * (1-correlation_coefficient) * (term1 + term2)
		measurement_uncertainty = math.sqrt(term1 + term2 + term3 + term4)
		### !!!!!!!!!!!! ###
		allometric_uncertainty.append(regression_uncertainty + measurement_uncertainty)
	allometric_uncertainty = numpy.array(allometric_uncertainty)
	###
	#print dbh
	#print allometric_uncertainty
	coefficients = numpy.polyfit(dbh,allometric_uncertainty,5)
	###
	polynomial = numpy.poly1d(coefficients)
	max_uncert = 0
	for i in xrange(len(dbh)):
		uncert = polynomial(dbh[i])
		if(uncert > max_uncert):
			max_uncert = uncert
	###
	return coefficients,max_uncert

def plotAllomUncert(fname):

#	fig_width_pt = 469.755
#	inches_per_pt = 1.0/72.27
#	golden_mean = (numpy.sqrt(5.0)-1.0)/2.0
#	fig_width = fig_width_pt*inches_per_pt*1
#	fig_height = fig_width*golden_mean
#	fig_size = [fig_width,fig_height]
#	params = {
#			"backend": "pdflatex",
#			"text.usetex": True,
#			"font.family": "serif",
#			"font.serif": [],
#			"font.sans-serif": [],
#			"font.monospace": [],
#			"axes.labelsize": 10,
#			"text.fontsize": 10,
#			"legend.fontsize": 8,
#			"xtick.labelsize": 8,
#			"ytick.labelsize": 8,
#			"figure.figsize": fig_size
#		}
#	matplotlib.rcParams.update(params)
	allomdata = sortAllomData(fname)
	fig,ax1 = matplotlib.pyplot.subplots()
	##residual error
	residuals = []
	for i in xrange(len(allomdata)):
		r = math.pow(numpy.log(allomdata[i][3])-numpy.log(allomdata[i][5]),2)
		residuals.append(r)
	residuals = numpy.array(residuals)
	ax1.scatter(allomdata['dbh'][:],residuals,color='k',marker='+',s=1)
	ax1.set_ylim([0,2])
	ax1.set_ylabel('RMSE')
	##regression uncertainty
	coefficients_std,max_uncertainty_std = allomUncert(fname,False,False)
	coefficients_dc,max_uncertainty_dc = allomUncert(fname,False,True)
	polynomial_std = numpy.poly1d(coefficients_std)
	polynomial_dc = numpy.poly1d(coefficients_dc)
	x_pts = numpy.linspace(0.1,1.5,100)
	ax2 = ax1.twinx()
	ax2.plot(x_pts,polynomial_std(x_pts),color='k',linestyle='--',label='Monte Carlo bootstrap regresssion perturbation')
	ax2.plot(x_pts,polynomial_dc(x_pts),color='k',label='above including DBH-constrained resample')
	ax2.set_ylim([0.5,0.8])
	ax2.set_xlim([0.1,2])
	ax2.set_ylabel('AGB estimation fractional uncertainty')
	ax1.set_xlabel('DBH (m)')
	ax2.legend(frameon=False)
	##
	matplotlib.pyplot.savefig('allometricUncertainty.pdf')

if __name__ == "__main__":

        parser = argparse.ArgumentParser()
        parser.add_argument('-dbh','--dbh',type=float,help='diameter m')
	parser.add_argument('-ht','--ht',type=float,help='height m')
        parser.add_argument('-wd','--wd',type=float,help='wood density kg/m3')
	parser.add_argument('-ad','--ad',default=False,help='allometry data, no uncertainty if omited')
	parser.add_argument('-mc','--mc',action='store_true',help='uncertainty via monte carlo bootsrapping')
	parser.add_argument('-dc','--dc',action='store_true',help='diamater classification')
	parser.add_argument('-pt','--pt',action='store_true',help='plot uncertainty')
        args = parser.parse_args()
	agb = chave2014(args.dbh,args.ht,args.wd)
	print "AGB",agb
	if(args.ad != False):
		coefficients,max_uncertainty = allomUncert(args.ad,args.mc,args.dc)
		polynomial = numpy.poly1d(coefficients)
		print "Uncertainty",polynomial(args.dbh)
		if(args.pt != False):
			plotAllomUncert(args.ad)

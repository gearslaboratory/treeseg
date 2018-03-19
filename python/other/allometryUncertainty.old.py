#!/usr/bin/env python

#Andrew Burt - a.burt.12@ucl.ac.uk

import argparse
import math
import numpy
import scipy.stats
import statsmodels.api
import matplotlib.pyplot

def sortData(fname):

	tmp = numpy.loadtxt(fname,dtype='S256')
	data = numpy.zeros(len(tmp),dtype=[('country','S256'),('D',float),('H',float),('AGB',float),('rho',float)])
	for i in xrange(len(tmp)):
		for j in xrange(len(tmp[0])):
			data[i][j] = tmp[i][j]
	return data

def resampleData(data,D_min=0,D_max=float("inf"),H_min=0,H_max=float("inf"),rho_min=0,rho_max=float("inf"),AGB_min=0,AGB_max=float("inf")):

	remove_list = []
	for i in xrange(len(data)):
		if(data[i]['D'] < D_min or data[i]['D'] > D_max or data[i]['H'] < H_min or data[i]['H'] > H_max or data[i]['rho'] < rho_min or data[i]['rho'] > rho_max or data[i]['AGB'] < AGB_min or data[i]['AGB'] > AGB_max):
			remove_list.append(i)
	data = numpy.delete(data,remove_list,axis=0)
	return data

def setCovariates():

	#definition; 1 list per covariate;
	#[model_id,[dependant power,log boolean,Y,Y_power],[[covariate power, log boolean, X1a, X1a_power, X1b, X1b_power , ..... , X1n, X1n_power],...,[covariate_n]]] 
	#i.e:
	#r'$\ln(D)$'								['M1',[1,1,'AGB',1],[[1,1,'D',1]]]
	#r'$\ln(D^2H)$'								['M2',[1,1,'AGB',1],[[1,1,'D',2,'H',1]]]
	#r'$\ln(D^2\rho)$'							['M3',[1,1,'AGB',1],[[1,1,'D',2,'rho',1]]]
	#r'$\ln(D^2H\rho)$'							['M4',[1,1,'AGB',1],[[1,1,'D',2,'H',1,'rho',1]]]
	#r'$\ln(D)+[\ln(D)]^2+[\ln(D)]^3$'					['M5',[1,1,'AGB',1],[[1,1,'D',1],[2,1,'D',1],[3,1,'D',1]]]
	#r'$\ln(D)+[\ln(D)]^2+[\ln(D)]^3+\ln(H)$'				['M6',[1,1,'AGB',1],[[1,1,'D',1],[2,1,'D',1],[3,1,'D',1],[1,1,'H',1]]]
	#r'$\ln(D)+[\ln(D)]^2+[\ln(D)]^3+\ln(\rho)$'				['M7',[1,1,'AGB',1],[[1,1,'D',1],[2,1,'D',1],[3,1,'D',1],[1,1,'rho',1]]]
	#r'$\ln(D)+[\ln(D)]^2+[\ln(D)]^3+\ln(H)+\ln(\rho)$'			['M8',[1,1,'AGB',1],[[1,1,'D',1],[2,1,'D',1],[3,1,'D',1],[1,1,'H',1],[1,1,'rho',1]]]
	#r'$\ln(D)+[\ln(D)]^2+[\ln(D)]^3+\ln(H)+[\ln(H)]^2+\ln(\rho)$'		['M9',[1,1,'AGB',1],[[1,1,'D',1],[2,1,'D',1],[3,1,'D',1],[1,1,'H',1],[2,1,'H',1],[1,1,'rho',1]]]
	cov = ['M9',[1,1,'AGB',1],[[1,1,'D',1],[2,1,'D',1],[3,1,'D',1],[1,1,'H',1],[2,1,'H',1],[1,1,'rho',1]]]
	return cov
 
def unpackCovariates(data,cov):

	Y = numpy.power(data[cov[1][2]],cov[1][3])
	if(cov[1][1] == 1):
		Y = numpy.log(Y)
	Y = numpy.power(Y,cov[1][0])
	X = []
	for i in xrange(len(data)):
		xi = numpy.ones(len(cov[2]))
		for j in xrange(len(cov[2])):
			k = 2
			while(k < len(cov[2][j])):
				xi[j] = xi[j] * numpy.power(data[i][cov[2][j][k]],cov[2][j][k+1])
				k += 2
			if(covariates[2][j][1] == 1):
				xi[j] = numpy.log(xi[j])
			xi[j] = numpy.power(xi[j],cov[2][j][0])
		X.append(xi)
	X = numpy.array(X)
	return Y,X

def generateModel(data,cov):
	
	Y,X = unpackCovariates(data,cov)
	X = statsmodels.api.add_constant(X)
	fit = statsmodels.api.OLS(Y,X)
	results = fit.fit()
	#print results.summary()
	model = []
	model.append(cov)
	coeffs = []	
	for i in xrange(len(results.params)):
		coeffs.append(results.params[i])
	model.append(coeffs)
	return model

def printAllomModel(model):

	model_id = model[0][0] 
	covariate_string = ''
	model_string = model[0][1][2]
	if(model[0][1][3] != 1):
		model_string += '^'+str(model[0][1][2])
	if(model[0][1][1] == 1):
		model_string = '\ln(' +model_string+')'
	if(model[0][1][0] != 1):
		model_string = '['+model_string+']^'+str(model[0][1][0])
	model_string = model_string+'='+"{:.3f}".format(model[1][0])+'+'
	for i in xrange(len(model[0][2])):
		covariate = ''
		j = 2
		while(j < len(model[0][2][i])):
			variable = model[0][2][i][j]
			if(variable == 'rho'):
				variable = r'\rho'
			covariate += variable
			if(model[0][2][i][j+1] != 1):
				covariate += '^'+str(model[0][2][i][j+1])
			j += 2
		if(model[0][2][i][1] == 1):
			covariate = '\ln('+covariate+')'
		if(model[0][2][i][0] != 1):
			covariate = '['+covariate+']^'+str(model[0][2][i][0])
		if(i == 0):
			covariate_string += covariate
			model_string += "{:.3f}".format(model[1][i+1])+covariate
		else:
			covariate_string += '+'+covariate
			model_string += '+'+"{:.3f}".format(model[1][i+1])+covariate
	return model_id,covariate_string,model_string

def predY(X,model,trans=False,see=0):

	Yp = model[1][0]
	for i in xrange(len(model[0][2])):
		Xpi = 1
		j = 2
		while(j < len(model[0][2][i])):
			Xpi = Xpi * math.pow(X[model[0][2][i][j]],model[0][2][i][j+1])
			j += 2	
		if(model[0][2][i][1] == 1):
			Xpi = math.log(Xpi)
		if(model[0][2][i][0] != 1):
			Xpi = math.pow(Xpi,model[0][2][i][0])
		Xpi = Xpi * model[1][i+1]
		Yp += Xpi
	if(trans == True and see != 0):
	 	Yp = math.exp(math.pow(see,2)/2) * math.exp(Yp)
	return Yp

#def nearestData(point,data,var,nnhb):
#
#	ndata = []
#	for i in xrange(nnhb):
#		idx = numpy.abs(data[variable]-point).argmin()
#		ndata.append(data[idx])
#		data = numpy.delete(data,idx,axis=0)
#	ndata = numpy.array(ndata)
#	return ndata

if __name__ == "__main__":

        parser = argparse.ArgumentParser()
	parser.add_argument('-d','--data',default=False,help='allometry data [id,d(cm),h(m),agb_obv(kg),rho(g/cm^3)')
        args = parser.parse_args()
	data = sortData(args.data)
	data = resampleData(data)
	covariates = setCovariates()
	model = generateModel(data,covariates)
	model_id,covariate_string,model_string = printAllomModel(model)
	print model_string
	print covariate_string
	###

	resid(data,model)

#def generateAllomModel(allomdata,exponant=True):
#
#	print 'ALLOMETRIC MODEL'	
#	#log transformation
#	lnagb = []
#	lnrhod2h = []
#	for i in xrange(len(allomdata)):
#		lnagb.append(math.log(allomdata[i][3]))
#		lnrhod2h.append(math.log(allomdata[i][4]/1000 * math.pow(allomdata[i][1]*100,2) * allomdata[i][2]))
#	lnagb = numpy.array(lnagb)
#	lnrhod2h = numpy.array(lnrhod2h)
#	#linear regression on logged data
#	if(exponant == False):
#		alpha = numpy.mean(lnagb - lnrhod2h)
#		beta = 1
#	if(exponant == True):
#		beta,alpha,r,p,stderr = scipy.stats.linregress(lnrhod2h,lnagb)
#	#fit sigma (required for log to normal space correction factor) 
#	residual_sum = 0
#	for j in xrange(len(lnagb)):
#		lnagb_est = alpha + beta * math.log(allomdata[j][4]/1000 * math.pow(allomdata[j][1]*100,2) * allomdata[j][2])
#		residual_sum += math.pow(lnagb[j] - lnagb_est,2)
#	if(exponant == False):
#		sigma = math.sqrt(residual_sum/(len(allomdata)-1))
#	if(exponant == True):
#		sigma = math.sqrt(residual_sum/(len(allomdata)-2))
#	#log - normal transformation
#	p1 = math.exp(math.pow(sigma,2)/2) * math.exp(alpha)
#	p2 = beta
#	print 'equation: agb_est = ' + str(p1) + ' (rho D^2 h)^' + str(p2)
#	print 'samples,sigma: '+str(len(allomdata))+','+str(sigma)
#	return alpha,beta,sigma

#def allomModelError(allomdata,alpha,beta,sigma):

#	print "MODEL ERROR: "
#	lnrhod2h = numpy.log(allomdata['wd']/1000 * numpy.power(allomdata['dbh']*100,2) * allomdata['h'])
#	###
#	intervals = 100
#	d_spacing = numpy.linspace(numpy.min(allomdata['dbh'])*100,numpy.max(allomdata['dbh'])*100,intervals)
#	h_spacing = numpy.linspace(numpy.min(allomdata['h']),numpy.max(allomdata['h']),intervals)
#	rho_spacing = numpy.linspace(numpy.min(allomdata['wd']/1000),numpy.max(allomdata['wd']/1000),intervals)
#	lnrhod2h_spacing = numpy.linspace(numpy.min(lnrhod2h),numpy.max(lnrhod2h),intervals)
#	lnrhod2h_spacing = numpy.linspace(4,numpy.max(lnrhod2h),intervals) ####TAKE NOTE OF THIS#####
#	agb_spacing = numpy.linspace(numpy.min(allomdata['agb_obv']),numpy.max(allomdata['agb_obv']),intervals)
#	###
#	DELTA_D = (numpy.max(allomdata['dbh'])*100 - numpy.min(allomdata['dbh']*100)) / 5
#	print "D: ",numpy.min(allomdata['dbh'])*100,numpy.max(allomdata['dbh'])*100,DELTA_D
#	DELTA_H = (numpy.max(allomdata['h']) - numpy.min(allomdata['h'])) / 5
#	print "H: ",numpy.min(allomdata['h']),numpy.max(allomdata['h']),DELTA_H
#	DELTA_RHO = (numpy.max(allomdata['wd']/1000) - numpy.min(allomdata['wd']/1000)) / 5
#	print "RHO: ",numpy.min(allomdata['wd']/1000),numpy.max(allomdata['wd']/1000),DELTA_RHO
#	DELTA_LNRHOD2H = (numpy.max(lnrhod2h) - numpy.min(lnrhod2h)) / 20
#	print "LNRHOD2H: ",numpy.min(lnrhod2h),numpy.max(lnrhod2h),DELTA_LNRHOD2H
#	DELTA_AGB = (numpy.max(allomdata['agb_obv']) - numpy.min(allomdata['agb_obv']))/15
#	print "AGB: ",numpy.min(allomdata['agb_obv']),numpy.max(allomdata['agb_obv']),DELTA_AGB
#	###
#	MIN_SAMPLES = 10
#	d_agb_obv,d_agb_est,h_agb_obv,h_agb_est,rho_agb_obv,rho_agb_est,lnrhod2h_agb_obv,lnrhod2h_agb_est,agb_agb_obv,agb_agb_est = [],[],[],[],[],[],[],[],[],[]
#	d_sxy,h_sxy,rho_sxy,lnrhod2h_sxy,agb_sxy = [],[],[],[],[]
#	d_stop,h_stop,rho_stop,lnrhod2h_stop,agb_stop = False,False,False,False,False
#	for i in xrange(intervals):
#		data = []
#		dbh_data,h_data,rho_data,lnrhod2h_data,agb_data = [],[],[],[],[]
#		for j in xrange(len(allomdata)):
#			if(allomdata[j][1]*100 > d_spacing[i]-DELTA_D/2 and allomdata[j][1]*100 < d_spacing[i]+DELTA_D/2 and d_stop == False):
#				dbh_data.append(allomdata[j])
#			if(allomdata[j][2] > h_spacing[i]-DELTA_H/2 and allomdata[j][2] < h_spacing[i]+DELTA_H/2 and h_stop == False):
#				h_data.append(allomdata[j])
#			if(allomdata[j][4]/1000 > rho_spacing[i]-DELTA_RHO/2 and allomdata[j][4]/1000 < rho_spacing[i]+DELTA_RHO/2 and rho_stop == False):
#				rho_data.append(allomdata[j])
#			if(lnrhod2h[j] > lnrhod2h_spacing[i]-DELTA_LNRHOD2H/2 and lnrhod2h[j] <= lnrhod2h_spacing[i]+DELTA_LNRHOD2H/2 and lnrhod2h_stop == False):
#				lnrhod2h_data.append(allomdata[j])
#			if(allomdata[j][3] > agb_spacing[i]-DELTA_AGB/2 and allomdata[j][3] <= agb_spacing[i]+DELTA_AGB/2 and agb_stop == False):
#				agb_data.append(allomdata[j])
#		if(len(dbh_data) < MIN_SAMPLES):
#			dbh_data = []
#			d_stop = True
#		if(len(h_data) < MIN_SAMPLES):
#			h_data = []
#			h_stop = True
#		if(len(rho_data) < MIN_SAMPLES):
#			rho_data = []
#			rho_stop = True
#		if(len(lnrhod2h_data) < 3):
#			lnrhod2h_data = []
#			lnrhod2h_stop = True
#		if(len(agb_data) < 3):
#			agb_data = []
#			agb_stop = True
#		if(d_stop != True):
#			agb_obv,agb_est = 0,0
#			residuals = 0
#			for k in xrange(len(dbh_data)):
#				agb_obv += dbh_data[k][3]
#				agb_est += math.exp(math.pow(sigma,2)/2) * math.exp(alpha) * math.pow(dbh_data[k][4]/1000 * math.pow(dbh_data[k][1]*100,2) * dbh_data[k][2],beta)
#				residuals += math.pow(math.log(dbh_data[k][3]) - (alpha + beta * math.log((dbh_data[k][4]/1000 * math.pow(dbh_data[k][1]*100,2) * dbh_data[k][2]))),2)
#			d_agb_obv.append(agb_obv) 
#			d_agb_est.append(agb_est)
#			d_sxy.append(math.sqrt(residuals / (len(dbh_data)-2)))
#		if(h_stop != True):
#			agb_obv,agb_est = 0,0
#			residuals = 0
#			for k in xrange(len(h_data)):
#				agb_obv += h_data[k][3]
#				agb_est += math.exp(math.pow(sigma,2)/2) * math.exp(alpha) * math.pow(h_data[k][4]/1000 * math.pow(h_data[k][1]*100,2) * h_data[k][2],beta)
#				residuals += math.pow(math.log(h_data[k][3]) - (alpha + beta * math.log((h_data[k][4]/1000 * math.pow(h_data[k][1]*100,2) * h_data[k][2]))),2)
#			h_agb_obv.append(agb_obv) 
#			h_agb_est.append(agb_est)
#			h_sxy.append(math.sqrt(residuals / (len(h_data)-2)))
#		if(rho_stop != True):
#			agb_obv,agb_est = 0,0
#			residuals = 0
#			for k in xrange(len(rho_data)):
#				agb_obv += rho_data[k][3]
#				agb_est += math.exp(math.pow(sigma,2)/2) * math.exp(alpha) * math.pow(rho_data[k][4]/1000 * math.pow(rho_data[k][1]*100,2) * rho_data[k][2],beta)
#				residuals += math.pow(math.log(rho_data[k][3]) - (alpha + beta * math.log((rho_data[k][4]/1000 * math.pow(rho_data[k][1]*100,2) * rho_data[k][2]))),2)
#			rho_agb_obv.append(agb_obv)
#			rho_agb_est.append(agb_est)
#			rho_sxy.append(math.sqrt(residuals / (len(rho_data)-2)))
#		if(lnrhod2h_stop != True):
#			agb_obv,agb_est = 0,0
#			residuals = 0
#			for k in xrange(len(lnrhod2h_data)):
#				agb_obv += lnrhod2h_data[k][3]
#				agb_est += math.exp(math.pow(sigma,2)/2) * math.exp(alpha) * math.pow(lnrhod2h_data[k][4]/1000 * math.pow(lnrhod2h_data[k][1]*100,2) * lnrhod2h_data[k][2],beta)
#				residuals += math.pow(math.log(lnrhod2h_data[k][3]) - (alpha + beta * math.log((lnrhod2h_data[k][4]/1000 * math.pow(lnrhod2h_data[k][1]*100,2) * lnrhod2h_data[k][2]))),2)
#			lnrhod2h_agb_obv.append(agb_obv)
#			lnrhod2h_agb_est.append(agb_est)
#			lnrhod2h_sxy.append(math.sqrt(residuals / (len(lnrhod2h_data)-2)))
#		if(agb_stop != True):
#			agb_obv,agb_est = 0,0
#			residuals = 0
#			for k in xrange(len(agb_data)):
#				agb_obv += agb_data[k][3]
#				agb_est += math.exp(math.pow(sigma,2)/2) * math.exp(alpha) * math.pow(agb_data[k][4]/1000 * math.pow(agb_data[k][1]*100,2) * agb_data[k][2],beta)
#				residuals += math.pow(math.log(agb_data[k][3]) - (alpha + beta * math.log((agb_data[k][4]/1000 * math.pow(agb_data[k][1]*100,2) * agb_data[k][2]))),2)
#			agb_agb_obv.append(agb_obv)
#			agb_agb_est.append(agb_est)
#			agb_sxy.append(math.sqrt(residuals / (len(agb_data)-2)))
#	d_agb_est = numpy.array(d_agb_est)
#	d_agb_obv = numpy.array(d_agb_obv)
#	d_sxy = numpy.array(d_sxy)
#	d_spacing = numpy.delete(d_spacing,numpy.arange(len(d_agb_obv),len(d_spacing),1,dtype=int),axis=0)
#	h_agb_est = numpy.array(h_agb_est)
#	h_agb_obv = numpy.array(h_agb_obv)
#	h_sxy = numpy.array(h_sxy)
#	h_spacing = numpy.delete(h_spacing,numpy.arange(len(h_agb_obv),len(h_spacing),1,dtype=int),axis=0)
#	rho_agb_est = numpy.array(rho_agb_est)
#	rho_agb_obv = numpy.array(rho_agb_obv)
#	rho_sxy = numpy.array(rho_sxy)
#	rho_spacing = numpy.delete(rho_spacing,numpy.arange(len(rho_agb_obv),len(rho_spacing),1,dtype=int),axis=0)
#	lnrhod2h_agb_est = numpy.array(lnrhod2h_agb_est)
#	lnrhod2h_agb_obv = numpy.array(lnrhod2h_agb_obv)
#	lnrhod2h_sxy = numpy.array(lnrhod2h_sxy)
#	lnrhod2h_spacing = numpy.delete(lnrhod2h_spacing,numpy.arange(len(lnrhod2h_agb_obv),len(lnrhod2h_spacing),1,dtype=int),axis=0)
#	agb_agb_est = numpy.array(agb_agb_est)
#	agb_agb_obv = numpy.array(agb_agb_obv)
#	agb_sxy = numpy.array(agb_sxy)
#	agb_spacing = numpy.delete(agb_spacing,numpy.arange(len(agb_agb_obv),len(agb_spacing),1,dtype=int),axis=0)
#	###
#	d_bias_coefficients = numpy.polyfit(d_spacing,((d_agb_est-d_agb_obv)/d_agb_obv),10)
#	d_bias_polynomial = numpy.poly1d(d_bias_coefficients)
#	d_sxy_coefficients = numpy.polyfit(d_spacing,d_sxy,10)
#	d_sxy_polynomial = numpy.poly1d(d_sxy_coefficients)
#	h_bias_coefficients = numpy.polyfit(h_spacing,((h_agb_est-h_agb_obv)/h_agb_obv),10)	
#	h_bias_polynomial = numpy.poly1d(h_bias_coefficients)
#	h_sxy_coefficients = numpy.polyfit(h_spacing,h_sxy,10)
#	h_sxy_polynomial = numpy.poly1d(h_sxy_coefficients)
#	rho_bias_coefficients = numpy.polyfit(rho_spacing,((rho_agb_est-rho_agb_obv)/rho_agb_obv),10)
#	rho_bias_polynomial = numpy.poly1d(rho_bias_coefficients)
#	rho_sxy_coefficients = numpy.polyfit(rho_spacing,rho_sxy,10)
#	rho_sxy_polynomial = numpy.poly1d(rho_sxy_coefficients)
#	lnrhod2h_bias_coefficients = numpy.polyfit(lnrhod2h_spacing,((lnrhod2h_agb_est-lnrhod2h_agb_obv)/lnrhod2h_agb_obv),5)	
#	lnrhod2h_bias_polynomial = numpy.poly1d(lnrhod2h_bias_coefficients)
#	lnrhod2h_sxy_coefficients = numpy.polyfit(lnrhod2h_spacing,lnrhod2h_sxy,5)
#	lnrhod2h_sxy_polynomial = numpy.poly1d(lnrhod2h_sxy_coefficients)
#	agb_bias_coefficients = numpy.polyfit(agb_spacing,((agb_agb_est-agb_agb_obv)/agb_agb_obv),10)	
#	agb_bias_polynomial = numpy.poly1d(agb_bias_coefficients)
#	agb_sxy_coefficients = numpy.polyfit(agb_spacing,agb_sxy,10)
#	agb_sxy_polynomial = numpy.poly1d(agb_sxy_coefficients)
#	###
#
#	sigma = 0.35754481624
#
#	cv = []
#	pi = []
#	cv_mbbc = []
#	pi_mbbc = []
#	agb = []
#
#	t_crit = scipy.stats.t.ppf(1-0.1, 4002)
#
#	sxx = 0
#	mean_lnrhod2h = numpy.mean(lnrhod2h)
#	for p in xrange(len(allomdata)):
#		sxx += math.pow(math.log(allomdata[i][4]/1000 * math.pow(allomdata[i][1]*100,2) * allomdata[i][2]) - mean_lnrhod2h,2)
#
#
#
#	for i in xrange(len(lnrhod2h)):
#		cov = math.sqrt(math.exp(math.pow(sigma,2))-1)
#		sigma_mbbc = lnrhod2h_sxy_polynomial(lnrhod2h[i])
#	 	cov_mbbc = math.sqrt(math.exp(math.pow(sigma_mbbc,2))-1)
#		pi = t_crit * sigma_mbbc * math.sqrt(1 + 1/4004 + math.pow(lnrhod2h[i] - mean_lnrhod2h,2) / sxx)
#		a = alpha + beta * lnrhod2h[i]
#		a_u = alpha + beta * lnrhod2h[i] + pi
#		a_l = alpha + beta * lnrhod2h[i] - pi
#		fu = (math.exp(a_u) - math.exp(a_l)) / math.exp(a_u)
#
#		agb.append(a)	
#		cv.append(cov)
#		cv_mbbc.append(cov_mbbc)
#		pi_mbbc.append(fu)
#
#	agb = numpy.array(agb)
#	cv = numpy.array(cv)
#	cv_mbbc = numpy.array(cv_mbbc)
#	pi_mbbc = numpy.array(pi_mbbc)
#	###
#	fig = matplotlib.pyplot.figure()
#	ax1 = fig.add_subplot(111)
#	ax2 = ax1.twinx()
#	ax3 = ax1.twinx()
#	fig.subplots_adjust(right=0.75)
#	ax3.spines['right'].set_position(('axes', 1.2))
#	ax3.set_frame_on(True)
#	ax3.patch.set_visible(False)
#	ax1.hist(lnrhod2h,numpy.arange(4,numpy.max(lnrhod2h),DELTA_LNRHOD2H),alpha=0.2,lw=0,color='k',label='Sample frequency; bin width')
#	ax2.scatter(lnrhod2h_spacing,((lnrhod2h_agb_est-lnrhod2h_agb_obv)/lnrhod2h_agb_obv),marker='+',s=5,color='k',alpha=0.5)
#	ax2.plot(lnrhod2h_spacing,lnrhod2h_bias_polynomial(lnrhod2h_spacing),linestyle='--',color='k',label='Model bias')
#	ax3.scatter(lnrhod2h_spacing,lnrhod2h_sxy,marker='x',s=5,color='k',alpha=0.5)
#	ax3.plot(lnrhod2h_spacing,lnrhod2h_sxy_polynomial(lnrhod2h_spacing),linestyle='-.',color='k',label='Model error')
#	ax1.set_xlabel(r'Predictor variable $(\ln(\rho D^2 h))$')
#	ax1.set_ylabel(r'Sample frequency $(N)$')
#	ax2.set_ylabel(r'Model bias $(\frac{(AGB_{est} - AGB_{obv})}{AGB_{obv}})$')
#	ax3.set_ylabel(r'Model error $(\sigma)$')
#	lines1, labels1 = ax1.get_legend_handles_labels()
#	lines2, labels2 = ax2.get_legend_handles_labels()
#	lines3, labels3 = ax3.get_legend_handles_labels()
#	ax1.legend(lines1+lines2+lines3,labels1+labels2+labels3,loc=0,frameon=False)
#	#ax1.set_ylim([0,4004])
#	ax1.set_xlim([4,16])
#	ax2.set_ylim([-0.4,0.4])
#	ax3.set_ylim([0.2,0.8])
#	fig.savefig('predictor_errorbias.pdf')
#	matplotlib.pyplot.close(fig)
#	####
#	fig = matplotlib.pyplot.figure()
#	ax1 = fig.add_subplot(111)
#	ax2 = ax1.twinx()
#	ax3 = ax1.twinx()
#	fig.subplots_adjust(right=0.75)
#	ax3.spines['right'].set_position(('axes', 1.2))
#	ax3.set_frame_on(True)
#	ax3.patch.set_visible(False)
#	ax1.hist(allomdata['dbh']*100,numpy.arange(numpy.min(allomdata['dbh']*100),numpy.max(allomdata['dbh']*100),DELTA_D),alpha=0.2,lw=0,color='k',label='Sample frequency; bin width')
#	ax2.scatter(d_spacing,((d_agb_est-d_agb_obv)/d_agb_obv),marker='+',s=5,color='k',alpha=0.5)
#	ax2.plot(d_spacing,d_bias_polynomial(d_spacing),linestyle='--',color='k',label='Model bias')
#	ax3.scatter(d_spacing,d_sxy,marker='x',s=5,color='k',alpha=0.5)
#	ax3.plot(d_spacing,d_sxy_polynomial(d_spacing),linestyle='-.',color='k',label='Model error')
#	ax1.set_xlabel(r'Diameter $(cm)$')
#	ax1.set_ylabel(r'Sample frequency $(N)$')
#	ax2.set_ylabel(r'Model bias $(\frac{(AGB_{est} - AGB_{obv})}{AGB_{obv}})$')
#	ax3.set_ylabel(r'Model error $(\sigma)$')
#	lines1, labels1 = ax1.get_legend_handles_labels()
#	lines2, labels2 = ax2.get_legend_handles_labels()
#	lines3, labels3 = ax3.get_legend_handles_labels()
#	ax1.legend(lines1+lines2+lines3,labels1+labels2+labels3,loc=0,frameon=False)
#	#ax1.set_ylim([0,4004])
#	ax1.set_xlim([0,200])
#	ax2.set_ylim([-0.4,0.4])
#	ax3.set_ylim([0.2,0.8])
#	fig.savefig('d_errorbias.pdf')
#	matplotlib.pyplot.close(fig)
#	###
#	fig = matplotlib.pyplot.figure()
#	ax1 = fig.add_subplot(111)
#	ax2 = ax1.twinx()
#	ax3 = ax1.twinx()
#	fig.subplots_adjust(right=0.75)
#	ax3.spines['right'].set_position(('axes', 1.2))
#	ax3.set_frame_on(True)
#	ax3.patch.set_visible(False)
#	ax1.hist(allomdata['h'],numpy.arange(numpy.min(allomdata['h']),numpy.max(allomdata['h']),DELTA_H),alpha=0.2,lw=0,color='k',label='Sample frequency; bin width')
#	ax2.scatter(h_spacing,((h_agb_est-h_agb_obv)/h_agb_obv),marker='+',s=5,color='k',alpha=0.5)
#	ax2.plot(h_spacing,h_bias_polynomial(h_spacing),linestyle='--',color='k',label='Model bias')
#	ax3.scatter(h_spacing,h_sxy,marker='x',s=5,color='k',alpha=0.5)
#	ax3.plot(h_spacing,h_sxy_polynomial(h_spacing),linestyle='-.',color='k',label='Model error')
#	ax1.set_xlabel(r'Height $(m)$')
#	ax1.set_ylabel(r'Sample frequency $(N)$')
#	ax2.set_ylabel(r'Model bias $(\frac{(AGB_{est} - AGB_{obv})}{AGB_{obv}})$')
#	ax3.set_ylabel(r'Model error $(\sigma)$')
#	lines1, labels1 = ax1.get_legend_handles_labels()
#	lines2, labels2 = ax2.get_legend_handles_labels()
#	lines3, labels3 = ax3.get_legend_handles_labels()
#	ax1.legend(lines1+lines2+lines3,labels1+labels2+labels3,loc=0,frameon=False)
	#ax1.set_ylim([0,4004])
#	ax1.set_xlim([0,70])
#	ax2.set_ylim([-0.4,0.4])
#	ax3.set_ylim([0.2,0.8])
#	fig.savefig('h_errorbias.pdf')
#	matplotlib.pyplot.close(fig)
#	###
#	fig = matplotlib.pyplot.figure()
#	ax1 = fig.add_subplot(111)
#	ax2 = ax1.twinx()
#	ax3 = ax1.twinx()
#	fig.subplots_adjust(right=0.75)
#	ax3.spines['right'].set_position(('axes', 1.2))
#	ax3.set_frame_on(True)
#	ax3.patch.set_visible(False)
#	ax1.hist(allomdata['wd']/1000,numpy.arange(numpy.min(allomdata['wd'])/1000,numpy.max(allomdata['wd'])/1000,DELTA_RHO),alpha=0.2,lw=0,color='k',label='Sample frequency; bin width')
#	ax2.scatter(rho_spacing,((rho_agb_est-rho_agb_obv)/rho_agb_obv),marker='+',s=5,color='k',alpha=0.5)
#	ax2.plot(rho_spacing,rho_bias_polynomial(rho_spacing),linestyle='--',color='k',label='Model bias')
#	ax3.scatter(rho_spacing,rho_sxy,marker='x',s=5,color='k',alpha=0.5)
#	ax3.plot(rho_spacing,rho_sxy_polynomial(rho_spacing),linestyle='-.',color='k',label='Model error')
#	ax1.set_xlabel(r'Wood density $(\frac{g}{cm^3})$')
#	ax1.set_ylabel(r'Sample frequency $(N)$')
#	ax2.set_ylabel(r'Model bias $(\frac{(AGB_{est} - AGB_{obv})}{AGB_{obv}})$')
#	ax3.set_ylabel(r'Model error $(\sigma)$')
#	lines1, labels1 = ax1.get_legend_handles_labels()
#	lines2, labels2 = ax2.get_legend_handles_labels()
#	lines3, labels3 = ax3.get_legend_handles_labels()
#	ax1.legend(lines1+lines2+lines3,labels1+labels2+labels3,loc=0,frameon=False)
#	#ax1.set_ylim([0,4004])
#	ax1.set_xlim([0,1.4])
#	ax2.set_ylim([-0.4,0.4])
#	ax3.set_ylim([0.2,0.8])
#	fig.savefig('rho_errorbias.pdf')
#	matplotlib.pyplot.close(fig)
#	###
#	fig = matplotlib.pyplot.figure()
#	ax1 = fig.add_subplot(111)
#	ax2 = ax1.twinx()
#	ax3 = ax1.twinx()
#	fig.subplots_adjust(right=0.75)
#	ax3.spines['right'].set_position(('axes', 1.2))
#	ax3.set_frame_on(True)
#	ax3.patch.set_visible(False)
#	ax1.hist(allomdata['agb_obv'],numpy.arange(numpy.min(allomdata['agb_obv']),numpy.max(allomdata['agb_obv']),DELTA_AGB),alpha=0.2,lw=0,color='k',label='Sample frequency; bin width')
#	ax2.scatter(agb_spacing,((agb_agb_est-agb_agb_obv)/agb_agb_obv),marker='+',s=5,color='k',alpha=0.5)
#	ax2.plot(agb_spacing,agb_bias_polynomial(agb_spacing),linestyle='--',color='k',label='Model bias')
#	ax3.scatter(agb_spacing,agb_sxy,marker='x',s=5,color='k',alpha=0.5)
#	ax3.plot(agb_spacing,agb_sxy_polynomial(agb_spacing),linestyle='-.',color='k',label='Model error')
#	ax1.set_xlabel(r'AGB $(kg)$')
#	ax1.set_ylabel(r'Sample frequency $(N)$')
#	ax2.set_ylabel(r'Model bias $(\frac{(AGB_{est} - AGB_{obv})}{AGB_{obv}})$')
#	ax3.set_ylabel(r'Model error $(\sigma)$')
#	lines1, labels1 = ax1.get_legend_handles_labels()
#	lines2, labels2 = ax2.get_legend_handles_labels()
#	lines3, labels3 = ax3.get_legend_handles_labels()
#	ax1.legend(lines1+lines2+lines3,labels1+labels2+labels3,loc=0,frameon=False)
#	#ax1.set_ylim([0,4004])
#	ax1.set_xlim([0,40000])
#	ax2.set_ylim([-0.4,0.4])
#	ax3.set_ylim([0.2,0.8])
#	fig.savefig('agb_errorbias.pdf')
#	matplotlib.pyplot.close(fig)
#	###
#	fig = matplotlib.pyplot.figure()
#	ax1 = fig.add_subplot(111)
#	ax1.scatter(numpy.exp(agb),cv,marker='.',s=5,color='k',label='Model CV')
#	ax1.scatter(numpy.exp(agb),cv_mbbc,marker='x',s=5,color='k',label='Monte-Carlo boostrap classification CV')
#	ax1.scatter(numpy.exp(agb),pi_mbbc,s=5,marker='*',color='k',label='Monte-Carlo boostrap classification PI')
#	ax1.set_xlabel(r'AGB $(kg)$')
#	ax1.set_ylabel(r'Fractional uncertainty')
#	ax1.legend(loc=0,frameon=False)
#	ax1.set_xlim([1000,60000])
#	ax1.set_ylim([0,1])
#	fig.savefig('agb_uncertainty.pdf')
#	matplotlib.pyplot.close(fig)
	###

#def plotParameters(allomdata):

#	m1 = numpy.log(allomdata['dbh']*100)
#	m2 = numpy.log(numpy.power(allomdata['dbh']*100,2)*allomdata['h'])
#	m3 = numpy.log(numpy.power(allomdata['dbh']*100,2)*allomdata['wd']/1000)
#	m4 = numpy.log(numpy.power(allomdata['dbh']*100,2)*allomdata['h']*allomdata['wd']/1000)
#
#	m5 = numpy.log(allomdata['dbh']) + numpy.power(numpy.log(allomdata['dbh']),2) + numpy.power(numpy.log(allomdata['dbh']),3)
#	m6 = numpy.log(allomdata['dbh']) + numpy.power(numpy.log(allomdata['dbh']),2) + numpy.power(numpy.log(allomdata['dbh']),3) + numpy.log(allomdata['h'])
#	m7 = numpy.log(allomdata['dbh']) + numpy.power(numpy.log(allomdata['dbh']),2) + numpy.power(numpy.log(allomdata['dbh']),3) + numpy.log(allomdata['wd'])
#	m8 = numpy.log(allomdata['dbh']) + numpy.power(numpy.log(allomdata['dbh']),2) + numpy.power(numpy.log(allomdata['dbh']),3) + numpy.log(allomdata['h']) + numpy.log(allomdata['wd'])
#
#	m8 =  numpy.log(allomdata['dbh']*100) + numpy.power(numpy.log(allomdata['dbh']*100),2)
#	m9 =  numpy.log(allomdata['dbh']*100) + numpy.power(numpy.log(allomdata['dbh']*100),2) + numpy.power(numpy.log(allomdata['dbh']*100),3)
#
#
#	m10 = numpy.log(allomdata['dbh']*100) + numpy.power(numpy.log(allomdata['dbh']*100),2) + numpy.power(numpy.log(allomdata['dbh']*100),3) + \
#		numpy.log(allomdata['h']) + numpy.power(numpy.log(allomdata['h']),2) + numpy.power(numpy.log(allomdata['h']),3) + numpy.log(allomdata['wd']/1000)
#
#
#	m11 = numpy.log(allomdata['dbh']*100) + numpy.power(numpy.log(allomdata['dbh']*100),2) + numpy.power(numpy.log(allomdata['dbh']*100),3) + \
#		numpy.log(allomdata['h']) + numpy.power(numpy.log(allomdata['h']),2) + numpy.power(numpy.log(allomdata['h']),3) + \
#		numpy.log(allomdata['wd']/1000) + numpy.power(numpy.log(allomdata['wd']/1000),2) + numpy.power(numpy.log(allomdata['wd']/1000),3)
#
#
 #
#
#	data = numpy.array([m8,m9,m10,m11])
#

#	data = numpy.array([numpy.log(numpy.power(allomdata['dbh'],1)),numpy.log(numpy.power(allomdata['dbh'],1) * numpy.power(allomdata['h'],1)),numpy.log(numpy.power(allomdata['wd'],1) * numpy.power(allomdata['dbh'],1)),numpy.log(numpy.power(allomdata['wd'],1) * numpy.power(allomdata['dbh'],1) * numpy.power(allomdata['h'],1)), \
#	numpy.log(allomdata['dbh'])+numpy.power(numpy.log(['dbh']),2),numpy.log(numpy.power(allomdata['dbh'],2) * numpy.power(allomdata['h'],1)),numpy.log(numpy.power(allomdata['wd'],1) * numpy.power(allomdata['dbh'],2)),numpy.log(numpy.power(allomdata['wd'],1) * numpy.power(allomdata['dbh'],2) * numpy.power(allomdata['h'],1)), \
#		numpy.log(numpy.power(allomdata['dbh'],3)),numpy.log(numpy.power(allomdata['dbh'],3) * numpy.power(allomdata['h'],1)),numpy.log(numpy.power(allomdata['wd'],1) * numpy.power(allomdata['dbh'],3)),numpy.log(numpy.power(allomdata['wd'],1) * numpy.power(allomdata['dbh'],3) * numpy.power(allomdata['h'],1))])
#	label = [r'$\ln(D)$',r'$\ln(D h)$',r'$\ln(\rho D)$',r'$\ln(\rho D h)$', \
#			r'$\ln(D^2)$',r'$\ln(D^2 h)$',r'$\ln(\rho D^2)$',r'$\ln(\rho D^2 h)$', \
#			r'$\ln(D^3)$',r'$\ln(D^3 h)$',r'$\ln(\rho D^3)$',r'$\ln(\rho D^3 h)$']
#	fig, ax = matplotlib.pyplot.subplots(2,4)
#	ax = ax.ravel()
#	for i in range(len(data)):
#		ax[i].scatter(data[i],numpy.log(allomdata['agb_obv']),alpha=0.5,s=0.25,lw=0,color='k')
#		ax[i].get_xaxis().set_ticks([])
#		ax[i].get_yaxis().set_ticks([])
#		ax[i].spines['top'].set_visible(False)
#		ax[i].spines['right'].set_visible(False)
#		ax[i].spines['bottom'].set_visible(False)
#		ax[i].spines['left'].set_visible(False)
#		ax[i].set_xlabel(label[i])
#	fig.text(0.02, 0.5,r'$\ln(AGB)$',ha='center',va='center',rotation='vertical')
#	fig.tight_layout()
#	fig.savefig('lnparams_lnagb.pdf')
#	matplotlib.pyplot.close(fig)
#

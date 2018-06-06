#!/usr/bin/env python

#Andrew Burt - a.burt.12@ucl.ac.uk

import argparse
import math
import numpy
import scipy.stats
#import statsmodels.api

#MODEL FORM:
#0 - id, 1 -lin(0)/nonlin(1)/spline(2), 2 - cov_str, 3 - model_str, 4 - covariates, 5 - regressand, 6 - X, 7 - y, 8 - beta	; dtype = nested list of lists
#[model_id,covariate_string,model_string,
#	[[covariate power, log boolean, X1a, X1a_power, X1b, X1b_power , ..... , X1n, X1n_power],...,[covariate_n]],
#		[regressand power,log boolean,Y,Y_power]
#			[X1, .... , XN]
#				[y1, ...., yn]
#					[beta_1, ... , beta_n]

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

def unpackModel(data,model):

	X = numpy.ones((len(data),len(model[4])+1))
	for i in xrange(len(data)):
		pos = 1
		for j in xrange(len(model[4])):
			xi = 1
			k = 2
			while(k < len(model[4][j])):
				xi = xi * math.pow(data[i][model[4][j][k]],model[4][j][k+1])
				k += 2 
			if(model[4][j][1] == 1):
				xi = math.log(xi)
			xi = math.pow(xi,model[4][j][0])
			X[i][pos] = xi
			pos += 1
	model[6] = numpy.ndarray.tolist(X)
	y = numpy.power(data[model[5][2]],model[5][3])
	if(model[5][1] == 1):
		y = numpy.log(y)
	y = numpy.power(y,model[5][0])
	model[7] = numpy.ndarray.tolist(y)
	return model

def printModel(model):

	covariate_string = ''
	model_string = model[5][2]
	if(model[5][3] != 1):
		model_string += '^'+str(model[5][2])
	if(model[5][1] == 1):
		model_string = '\ln(' +model_string+')'
	if(model[5][0] != 1):
		model_string = '['+model_string+']^'+str(model[5][0])
	model_string = model_string+'='+"{:.3f}".format(model[8][0])+'+'
	for i in xrange(len(model[4])):
		cov = ''
		j = 2
		while(j < len(model[4][i])):
			var = model[4][i][j]
			if(var == 'rho'):
				var = r'\rho'
			if(model[4][i][j+1] != 1):
				var += '^'+str(model[4][i][j+1])
			cov += var
			j += 2
		if(model[4][i][1] == 1):
			cov = '\ln('+cov+')'
		if(model[4][i][0] != 1):
			cov = '['+cov+']^'+str(model[4][i][0])
		if(i == 0):
			covariate_string += cov
			model_string += "{:.3f}".format(model[8][i+1])+cov
		else:
			covariate_string += '+'+cov
			model_string += '+'+"{:.3f}".format(model[8][i+1])+cov
	model[2] = covariate_string
	model[3] = model_string

def linearModel(data,model):
	
	model = unpackModel(data,model)
	model[8] = numpy.ndarray.tolist(numpy.dot(numpy.dot(numpy.linalg.matrix_power(numpy.dot(numpy.matrix.transpose(numpy.array(model[6])),numpy.array(model[6])),-1),numpy.matrix.transpose(numpy.array(model[6]))),numpy.array(model[7])))
	printModel(model)
#	model = statsmodels.api.OLS(model[7],model[6])
#	results = model.fit()
#	print results.summary()
#	print "linearity:"
#	print "rainbow: ",statsmodels.api.stats.diagnostic.linear_rainbow(results)
#	print "homoscedasticity:"
#	print "breushpagan: ",statsmodels.api.stats.diagnostic.het_breushpagan(results.resid,X)
#	print "white: ",statsmodels.api.stats.diagnostic.het_white(results.resid,X)
	return model

def yhat(X0,model,inverse=False):

	if(inverse == False):
		return numpy.dot(X0,numpy.array(model[8]))
	else:
		sigma = ser(model)
		return numpy.exp(numpy.dot(X0,numpy.array(model[8]))) #* math.exp(math.pow(sigma,2)/2)

def resid(model):

	return numpy.array(model[7]) - numpy.array(numpy.dot(model[6],model[8]))

def ser(model):

	return math.sqrt(numpy.dot(numpy.matrix.transpose(numpy.array(model[7])),resid(model)) / (len(numpy.array(model[7])) - len(numpy.array(model[8]))))

def pi(X0,X,y,model,alpha=0.05):

	yp = yhat(X0,model) 
	yp_l = numpy.zeros(len(X0))
	yp_u = numpy.zeros(len(X0))
	sigma = ser(y,X,model)
	tppf = scipy.stats.t.isf(alpha/2.,(len(y)-len(model[1])))
	mp = numpy.linalg.matrix_power(numpy.dot(numpy.matrix.transpose(X),X),-1)
	for i in xrange(len(X0)):
		yp_l[i] = yp[i] - tppf * math.sqrt(sigma * (1 + numpy.dot(numpy.dot(numpy.matrix.transpose(X0[i]),mp),X[i])))
		yp_u[i] = yp[i] + tppf * math.sqrt(sigma * (1 + numpy.dot(numpy.dot(numpy.matrix.transpose(X0[i]),mp),X[i])))
	return yp,yp_l,yp_u

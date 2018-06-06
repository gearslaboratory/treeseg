#!/usr/bin/env python

#Andrew Burt - a.burt.12@ucl.ac.uk

import argparse
import math
import numpy
import scipy.optimize
import random
import copy

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

	X = numpy.ones(len(data))
	for i in xrange(len(data)):
		for j in xrange(len(model[4])):
			xi = 1
			k = 2
			while(k < len(model[4][j])):
				xi = xi * math.pow(data[i][model[4][j][k]],model[4][j][k+1])
				k += 2 
			if(model[4][j][1] == 1):
				xi = math.log(xi)
			xi = math.pow(xi,model[4][j][0])
			X[i] = xi
	model[6] = numpy.ndarray.tolist(X)
	y = numpy.power(data[model[5][2]],model[5][3])
	if(model[5][1] == 1):
		y = numpy.log(y)
	y = numpy.power(y,model[5][0])
	model[7] = numpy.ndarray.tolist(y)
	return model

def printModel(model):

	model[2] = r'D^2H\rho'
	model[3] = 'AGB = '+"{:.3f}".format(model[8][0])+r'(D^2H\rho)^'+"{:.3f}".format(model[8][1])
	return model

def func(X,a,b,c):

	return a*numpy.power(X,b)+c

def nonlinearModel(data,model):

	model = unpackModel(data,model)
	coeff,vcvm = scipy.optimize.curve_fit(func,numpy.array(model[6]),numpy.array(model[7]))
	model[8] = numpy.ndarray.tolist(coeff)
	model = printModel(model)
	return model

def yhat(X0, model):

	return model[8][0] * numpy.power(X0,model[8][1]) + model[8][2]

def resid(model):

	return numpy.array(model[7]) - model[8][0] * numpy.power(numpy.array(model[6]),model[8][1])

def wildbootstrap(data,model,runs):

	models = []
	for i in xrange(runs):
		obs = numpy.random.randint(10,high=len(data)) #between 5 and 4003#
		idx = numpy.random.randint(0,high=len(data),size=obs)
		boot_data = numpy.zeros(len(idx),dtype=[('country','S256'),('D',float),('H',float),('AGB',float),('rho',float)])
		for j in xrange(len(idx)):
			boot_data[j] = data[idx[j]]
		try:
			models.append(copy.copy(nonlinearModel(boot_data,copy.copy(model))))

		except RuntimeError:
			continue
	return models

def ci(data,model,x_range,runs):

	models = wildbootstrap(data,model,runs)
	y_range = numpy.zeros((len(x_range),len(models)))
	for i in xrange(len(models)):
		y_range[:,i] = yhat(x_range[:],models[i])
	y_range = numpy.sort(y_range,axis=1)
	y_cil = numpy.percentile(y_range,5,axis=1)
	y_ciu = numpy.percentile(y_range,95,axis=1)
	return y_cil,y_ciu

def pi(data,model,x_range,runs):

#	h = 0
#	b =scipy.stats.mstats.mquantiles(numpy.array(model[6]), prob=0.95, alphap=1/3, betap=1/3, axis=None, limit=(h))
	
	b = scipy.stats.mstats.mquantiles(numpy.array(model[6]))
	
	print b

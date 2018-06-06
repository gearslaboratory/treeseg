#!/usr/bin/env python

#Andrew Burt - a.burt.12@ucl.ac.uk

import argparse
import math
import numpy
import scipy.interpolate
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
	model[3] = 'spline'
	return model

def splineModel(data,model):

	model = unpackModel(data,model)
	order = numpy.argsort(numpy.array(model[6]))
#	model[8] = scipy.interpolate.UnivariateSpline(numpy.array(model[6])[order],numpy.array(model[7])[order],s=7500000000)
	model[8] = scipy.interpolate.UnivariateSpline(numpy.array(model[6])[order],numpy.array(model[7])[order],s=6000000000)
	model = printModel(model)
	return model

def yhat(X0,model):

	#dont want spline extrapolation
#	for i in xrange(len(X0)):
#		if(X0[i] < numpy.min(numpy.array(model[6])) or X0[i] > numpy.max(numpy.array(model[6]))):
#			X0[i] = numpy.nan
#	X0[(X0 < numpy.min(numpy.array(model[6]))) | (X0 > numpy.max(numpy.array(model[6])))] = numpy.nan
	return model[8](X0)

def resid(model):

	return numpy.array(model[7]) - model[8](numpy.array(model[6]))

def wildbootstrap(data,model,runs):

	models = []
	for i in xrange(runs):
		obs = numpy.random.randint(5,high=len(data)) #between 5 and 4003#
		idx = numpy.random.randint(0,high=len(data),size=obs)
		boot_data = numpy.zeros(len(idx),dtype=[('country','S256'),('D',float),('H',float),('AGB',float),('rho',float)])
		for j in xrange(len(idx)):
			boot_data[j] = data[idx[j]]
		models.append(copy.copy(splineModel(boot_data,copy.copy(model)))) #beware of self
	return models

#def ci(data,model,runs):
#
#	models = wildbootstrap(data,model,runs)
#	xnew = numpy.linspace(numpy.min(numpy.array(model[6])),numpy.max(numpy.array(model[6])),10000)
#	ynew = numpy.zeros((len(xnew),len(models)))
#	for i in xrange(len(models)):
#		ynew[:,i] = yhat(xnew[:],models[i])
#	ynew = numpy.sort(ynew,axis=1)
#	y_pred = yhat(xnew,model)
#	y_cil = numpy.nanpercentile(ynew,5,axis=1)
#	y_ciu = numpy.nanpercentile(ynew,95,axis=1)
#	return xnew,y_pred,y_cil,y_ciu

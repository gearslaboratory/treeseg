#!/usr/bin/env python

import sys
import numpy as np
import math

def stddev(observed,modelled):

	sum = 0
	for i in xrange(len(observed)):
		sum += math.pow(observed[i]-modelled[i],2)
	stddev = math.sqrt(sum/(len(observed)-1))
	return stddev

def rsquared(observed,modelled):

	ss_tot = 0
	ss_res = 0
	for i in xrange(len(observed)):
		ss_tot += math.pow(observed[i] - np.mean(observed),2)
	for j in xrange(len(modelled)):
		ss_res += math.pow(observed[j] - modelled[j],2)
	r_sqaured = 1 - (ss_res / ss_tot)
	return r_sqaured

def Chave2014(data):

	#Fit TLS data to the Chave 2014 equation
	alpha = -2.949
	log_agb_obv = np.log(data['lb'])
	log_agb_est = alpha + np.log(data['ldbh']*data['ldbh']*data['lh']*data['wd']*10)
	std = stddev(log_agb_obv,log_agb_est)
	r2 = rsquared(log_agb_obv,log_agb_est)
	print 'Chave2014',alpha,std,r2

def Chave2014noheight(data):

	#Fit TLS data Chave 2014 height inferred equation
	alpha = -2.949
	log_agb_obv = np.log(data['lb'])
	log_agb_est = np.zeros((len(log_agb_obv)))
	for i in xrange(len(data)):
		TS = 1
		CWD = 1
		PS = 1
		#E = (0.178 * TS -0.983 * CWD - 6.61 * PS) * 0.001
		E = -0.1
		log_h = 0.893 - E + 0.76 * np.log(data[i]['ldbh']*100) - 0.0340 * math.pow(np.log(data[i]['ldbh']*100),2)
		log_agb_est = alpha + np.log(data['ldbh']*data['ldbh']*data['wd']*10) + log_h
	std = stddev(log_agb_obv,log_agb_est)
	r2 = rsquared(log_agb_obv,log_agb_est)
	print 'Chave2014 w/o height',alpha,std,r2

def fitOwn(data):

	#Fit TLS data to rho*d^2*h
	alpha_range = [-10,10]
	log_agb_obv = np.log(data['lb'])
	c = alpha_range[0]
	c_increment = 0.001
	answers = []
	i = 0
	while (c < alpha_range[1]):
		log_agb_est = c + np.log(data['ldbh']*data['ldbh']*data['lh']*data['wd']*10)
		std = stddev(log_agb_obv,log_agb_est)
		answers.append([c,std])
		c += c_increment
	answers = np.array(answers)
	idx = np.argmin(answers[:,1])
	alpha = answers[idx][0]
	log_agb_est = alpha + np.log(data['ldbh']*data['ldbh']*data['lh']*data['wd']*10)
	std = stddev(log_agb_obv,log_agb_est)
	r2 = rsquared(log_agb_obv,log_agb_est)
	print 'Own',alpha,std,r2

def fitEH(data):

	#Fit TLS data to eH
	alpha_range = [-10,10]
	log_agb_obv = np.log(data['lb'])
	c = alpha_range[0]
	c_increment = 0.001
	answers = []
	i = 0
	while (c < alpha_range[1]):
		log_agb_est = c + np.log(data['le']*data['lh'])
		std = stddev(log_agb_obv,log_agb_est)
		answers.append([c,std])
		c += c_increment
	answers = np.array(answers)
	idx = np.argmin(answers[:,1])
	alpha = answers[idx][0]
	log_agb_est = alpha + np.log(data['le']*data['lh'])
	std = stddev(log_agb_obv,log_agb_est)
	r2 = rsquared(log_agb_obv,log_agb_est)
	print 'eH',alpha,std,r2

tmp = np.load(sys.argv[1])
data = tmp['arr_0']
#Chave2014(data)
#Chave2014noheight(data)
#fitOwn(data)
fitEH(data)

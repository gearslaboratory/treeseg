#!/usr/bin/env python

#Andrew Burt - a.burt.12@ucl.ac.uk

import argparse
import numpy
import oldallometry

def mergeData(cloud_data,model_data):

	c_data = numpy.loadtxt(cloud_data,dtype='S256')
	m_data = numpy.loadtxt(model_data,dtype='S256')
	results = numpy.zeros(len(m_data),dtype=[('lid','S256'),('lx',float),('ly',float),('ldbh',float),('lh',float),('lv',float),('lv_fu',float),('lwd',float),('lagb_vol',float),('lagb_vol_fu',float),('lagb_allom',float),('lagb_allom_fu',float)])
	for i in xrange(len(m_data)): #length of lowest count array
		results['lid'][i] = m_data[i][0]
		results['lv'][i] = m_data[i][1]
		results['lv_fu'][i] = m_data[i][2]
		for j in xrange(len(c_data)):
			if(c_data[j][0] == results['lid'][i]):
				results['lx'][i] = float(c_data[j][1])
				results['ly'][i] = float(c_data[j][2])
				results['ldbh'][i] = float(c_data[j][3])
				results['lh'][i] = float(c_data[j][4])
				break
	return results

def getResults(results,plot_density,allom_fname):

	coefficients,max_uncertainty = oldallometry.allomUncert(allom_fname,False,True)
	polynomial = numpy.poly1d(coefficients)
	for i in xrange(len(results)):
		results['lwd'][i] = plot_density
		results['lagb_vol'][i] = results['lv'][i] * results['lwd'][i]
		results['lagb_vol_fu'][i] = results['lv_fu'][i]
		results['lagb_allom'][i] = oldallometry.chave2014(results['ldbh'][i],results['lh'][i],results['lwd'][i])
		results['lagb_allom_fu'][i] = min(polynomial(results['ldbh'][i]),max_uncertainty)

def printResults(results):
	
	print results
	agb_vol = numpy.sum(results['lagb_vol'])
	agb_allom = numpy.sum(results['lagb_allom'])
	diff = min(agb_vol,agb_allom)/max(agb_vol,agb_allom)
	print "AGB vol,allom,%diff:",agb_vol,agb_allom,1-diff

def saveResults(results,fname):

	numpy.save(fname,results)

if __name__ == "__main__":

	parser = argparse.ArgumentParser()
	parser.add_argument('-c','--cloud_data',default=False)
	parser.add_argument('-m','--model_data',default=False)
	parser.add_argument('-d','--plot_density',default=650,help='plot density (kg/m3)')
	parser.add_argument('-a','--allom_data',default=False)
	parser.add_argument('-o','--out_file',default='results.npy')
	args = parser.parse_args()
	results = mergeData(args.cloud_data,args.model_data)
	getResults(results,args.plot_density,args.allom_data)
	printResults(results)
	saveResults(results,args.out_file)

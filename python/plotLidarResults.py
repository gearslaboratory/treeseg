#!/usr/bin/env python

#Andrew Burt - a.burt.12@ucl.ac.uk

import argparse
import numpy
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot
import itertools

def plotResults(fnames):

	fig_width_pt = 418.82394
	inches_per_pt = 1.0/72.27
	golden_mean = (numpy.sqrt(5.0)-1.0)/2.0
	fig_width = fig_width_pt*inches_per_pt*1
#	fig_height = fig_width*golden_mean
	fig_height = fig_width
	fig_size = [fig_width,fig_height]
	params = {
			"backend": "pdflatex",
			"text.usetex": True,
			"font.family": "serif",
			"font.serif": [],
			"font.sans-serif": [],
			"font.monospace": [],
			"axes.labelsize": 10,
			"font.size": 10,
			"legend.fontsize": 8,
			"xtick.labelsize": 8,
			"ytick.labelsize": 8,
			"figure.figsize": fig_size
		}
	matplotlib.rcParams.update(params)
	marker = itertools.cycle(('+', '.', '^', '*'))
	for i in xrange(len(fnames)):
		data = numpy.load(fnames[i])
		lid = data['lid'][0].split('_')[0]
		matplotlib.pyplot.scatter(numpy.log(data['lagb_allom']),numpy.log(data['lagb_vol']),label=lid,marker=marker.next(),color='k')
		print sum(data['lagb_allom'])/sum(data['lagb_vol'])
	x = [4,11]
	y = [4,11]
	matplotlib.pyplot.plot(x,y,label='1:1',color='k')
	matplotlib.pyplot.xlabel('ln(AGB from TLS derived allometry) (kg)')
	matplotlib.pyplot.ylabel('ln(AGB from TLS dervied volume) (kg)')
	matplotlib.pyplot.xlim([4,11])
	matplotlib.pyplot.ylim([4,11])
	matplotlib.pyplot.legend(loc=4,frameon=False)
	matplotlib.pyplot.savefig('tlsresults.pdf')

if __name__ == "__main__":

	parser = argparse.ArgumentParser()
	parser.add_argument('-r','--results',nargs='*',default=False,help='list of numpy results files')
	args = parser.parse_args()
	plotResults(args.results)

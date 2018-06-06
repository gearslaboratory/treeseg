#!/usr/bin/env python

#Andrew Burt - a.burt.12@ucl.ac.uk

import argparse
import numpy
import scipy.interpolate
import scipy.stats
import matplotlib.pyplot
import mpl_toolkits.axes_grid1.inset_locator

import allometry_linear
import allometry_nonlinear

#fig_width_pt = 418.82394
#inches_per_pt = 1.0/72.27
#golden_mean = (numpy.sqrt(5.0)-1.0)/2.0
#fig_width = fig_width_pt*inches_per_pt*1
#fig_height = fig_width*golden_mean
#fig_size = [fig_width,fig_height]
#params = {
#		"backend": "pdflatex",
#		"text.usetex": True,
#		"font.family": "serif",
#		"font.serif": [],
#		"font.sans-serif": [],
#		"font.monospace": [],
#		"axes.labelsize": 10,
#		"font.size": 10,
#		"legend.fontsize": 8,
#		"xtick.labelsize": 8,
#		"ytick.labelsize": 8,
#		"figure.figsize": fig_size
#}
#matplotlib.rcParams.update(params)

def plotRes(model):

	fig = matplotlib.pyplot.figure()
	ax = fig.add_subplot(111)
	if(model[1] == 0):
		residuals = allometry_linear.resid(model)
		m = numpy.polyfit(numpy.array(model[7]),residuals,1)
		mr = numpy.poly1d(m)
		x_new=numpy.linspace(2,7.5,100)
		ax.hist2d(numpy.array(model[7]),residuals,bins=40,cmap='gray_r')
		#ax.scatter(numpy.array(model[7]),residuals,alpha=0.33,s=10,lw=0,color='k')
		ax.plot([0,12],[0,0],alpha=1,linewidth=1,linestyle='--',color='k')
		ax.plot(x_new,mr(x_new),alpha=1,linewidth=1,color='k')
		ax.set_ylim([-2,2])
		ax.set_xlim([0,12])
	elif(model[1] == 1):
		residuals = allometry_nonlinear.resid(model)
		ax.scatter(numpy.array(model[7]),residuals,alpha=0.33,s=10,lw=0,color='k')
	elif(model[1] == 2):
		residuals = allometry_spline.resid(model)
		ax.scatter(numpy.array(model[7]),residuals,color='k',alpha=0.33,s=10,lw=0)
	ax.set_ylabel('Residuals')
	ax.set_xlabel(r'\ln(AGB)')
	matplotlib.pyplot.savefig(model[0]+'_residuals.pdf')
	matplotlib.pyplot.close()	

def plotQQ(model):

	if(model[1] == 0):
		residuals = allometry_linear.resid(model)
	else:
		residuals = allometry_nonlinear.resid(model)
	plotting_pos = (numpy.arange(1.,residuals.shape[0]+1) - 0)/(residuals.shape[0]- 2*0 + 1)
	b = scipy.stats.norm.ppf(plotting_pos)
	fit_params = scipy.stats.norm.fit(residuals)
	loc = fit_params[-2]
	scale = fit_params[-1]
	sorted_data = numpy.array(residuals, copy=True)
	sorted_data.sort()
	sorted_data = (sorted_data-loc)/scale
	fig = matplotlib.pyplot.figure()
	ax = fig.add_subplot(111)
	ax.scatter(b,sorted_data,alpha=0.33,s=10,lw=0,color='k')
	ax.plot([-5,5],[-5,5],color='k',alpha=0.33,linestyle='--',linewidth=1)
	ax.set_xlim([-5,5])
	ax.set_ylim([-5,5])
	axins = mpl_toolkits.axes_grid1.inset_locator.zoomed_inset_axes(ax, 1.5, loc=4)
	axins.scatter(b,sorted_data,alpha=0.33,s=2,lw=0,color='k')
	axins.plot([-5,5],[-5,5],color='k',alpha=0.33,linestyle='--',linewidth=1)
	axins.set_xlim(-1.5, 1.5)
	axins.set_ylim(-1.5, 1.5)
	axins.set_xticklabels([])
	axins.set_yticklabels([])
	ax.set_xlabel('Theoretical Quantile')
	ax.set_ylabel('Sample Quantile')
	mpl_toolkits.axes_grid1.inset_locator.mark_inset(ax, axins, loc1=1, loc2=3, fc="none", ec="0.5")
	matplotlib.pyplot.savefig(model[0]+'_QQ.pdf')
	matplotlib.pyplot.close()

def plotHist(model):

	if(model[1] == 0):
		residuals = allometry_linear.resid(model)
		bins = numpy.linspace(-3,3,31)
		fig = matplotlib.pyplot.figure()
		ax = fig.add_subplot(111)
		ax.hist(residuals,bins,normed=1,color='k',alpha=0.3,linewidth=0)
		mean = 0
		sigma = allometry_linear.ser(model)
		x = numpy.linspace(-3,3,100)
		ax.plot(x,scipy.stats.norm.pdf(x,mean,sigma),color='k',alpha=0.33,linestyle='--',linewidth=1)
		ax.set_xlim([-2,2])
		ax.set_ylim([0,1.2])
		ax.set_xlabel('Residuals')
		ax.set_ylabel('Probability density')
		matplotlib.pyplot.savefig(model[0]+'_hist.pdf')
		matplotlib.pyplot.close()

def nearestData(point,model,neighbours):

	nX = []
	ny = []
	X = numpy.array(model[6])
	y = numpy.array(model[7])
	for i in xrange(neighbours):
		idx = numpy.abs(y-point).argmin()
		nX.append(X[idx])
		ny.append(y[idx])
		X = numpy.delete(X,idx,axis=0)
		y = numpy.delete(y,idx,axis=0)
	nX = numpy.array(nX)
	ny = numpy.array(ny)
	return nX,ny

def plotBias(data,model,models,neighbours):

	xnew = numpy.linspace(numpy.min(numpy.array(model[6])),numpy.max(numpy.array(model[6])),1000)
	fig = matplotlib.pyplot.figure()
	ax = fig.add_subplot(111)
	ax.plot(xnew,model[8](xnew))
	ax.plot(xnew,allometry_nonlinear.yhat(xnew,models[1]))
	ax.scatter(model[6],model[7])
	matplotlib.pyplot.savefig('model.pdf')
	matplotlib.pyplot.close(fig)


	bias1 = []
	bias2 = []
	ynew = numpy.linspace(numpy.min(numpy.array(model[7])),numpy.max(numpy.array(model[7])),1000)
	for i in xrange(len(ynew)):
		nX,ny = nearestData(ynew[i],models[0],50)
		nyp = allometry_spline.yhat(nX,model)
		bias1.append(numpy.mean((nyp - ny) / ny))


		nX,ny = nearestData(ynew[i],models[1],50)
		nyp = allometry_nonlinear.yhat(nX,models[1])
		bias2.append(numpy.mean((nyp - ny) / ny))



	fig = matplotlib.pyplot.figure()
	ax = fig.add_subplot(111)
	ax.plot(ynew,bias1,color='k')
	ax.plot(ynew,bias2,color='k')
	ax.plot([0,30000],[0,0],color='k',alpha=0.33,linestyle='--',linewidth=1)
	ax.set_xlim([0,100000])
	ax.set_ylim([-0.4,0.4])
	ax.set_xlabel(r'$AGB (kg)$')
	ax.set_ylabel('Model bias')
	matplotlib.pyplot.savefig('bias.pdf')
	matplotlib.pyplot.close()

def plotModel(data,model):

	fig = matplotlib.pyplot.figure()
	ax = fig.add_subplot(111)
	ax.scatter(numpy.array(model[6]),numpy.array(model[7]))
	xnew = numpy.linspace(numpy.min(numpy.array(model[6])),numpy.max(numpy.array(model[6])),1000)
	ax.plot(xnew,allometry_nonlinear.yhat(xnew,model))
	y_cil,y_ciu = allometry_nonlinear.ci(data,model,xnew,100)
	ax.plot(xnew,y_cil)
	ax.plot(xnew,y_ciu)

	ax.set_xlim([0,1400000])
	ax.set_ylim([0,100000])

	matplotlib.pyplot.savefig('model.pdf')
	matplotlib.pyplot.close(fig)

if __name__ == "__main__":

        parser = argparse.ArgumentParser()
	parser.add_argument('-d','--data',default=False,help='allometry data [id,d(cm),h(m),agb_obv(kg),rho(g/cm^3)')
        args = parser.parse_args()

	data = allometry_linear.sortData(args.data)
	data = allometry_linear.resampleData(data)

	m1 = ['M1',0,'','',[[1,1,'D',2,'H',1,'rho',1]],[1,1,'AGB',1],[],[],[]]
	m2 = ['M2',0,'','',[[1,1,'D',1],[2,1,'D',1],[3,1,'D',1],[1,1,'H',1],[1,1,'rho',1]],[1,1,'AGB',1],[],[],[]]
	m3 = ['M3',1,'','',[[1,0,'D',2,'H',1,'rho',1]],[1,0,'AGB',1],[],[],[]]

	m1 = allometry_linear.linearModel(data,m1)
	m2 = allometry_linear.linearModel(data,m2)
	m3 = allometry_nonlinear.nonlinearModel(data,m3)

	allometry_nonlinear.pi(data,m3,0,0)

	#plotModel(data,m3)

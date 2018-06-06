#!/usr/bin/env python

#Andrew Burt - a.burt.12@ucl.ac.uk

import argparse
import glob
import os
import numpy
import math
import scipy.io
import scipy.stats
import shutil
import multiprocessing
import itertools
import plotTrees

def getResults(model_names,min_diameter):

	lid = model_names[0].split('/')[len(model_names[0].split('/'))-1].split('-')[0]
	volume = []
	for i in xrange(len(model_names)):
		try:
			model = scipy.io.loadmat(model_names[i])
			model_volume = 0
			for j in xrange(len(model['Rad'])):
				if(model['Rad'][j][0] * 2 >= min_diameter):
					model_volume += math.pi * model['Rad'][j][0] * model['Rad'][j][0] * model['Len'][j][0]
			volume.append(model_volume)
		except:
			pass
	volume = numpy.array(volume)
	mean = numpy.mean(volume)
	stddev = numpy.std(volume)
	#sem = scipy.stats.sem(volume)
	#interval = sem * scipy.stats.t._ppf((1+0.95)/2.,len(volume)-1)
	idx = numpy.argmin(numpy.abs(volume[:]-mean))
	best_model = model_names[idx]
	results = numpy.zeros(1,dtype=[('lid','S256'),('lv',float),('lv_fu',float)])
	results['lid'][0] = lid
	results['lv'][0] = mean
	results['lv_fu'][0] = (stddev * 2) / mean
	return results,best_model

def getVolume(model_names,min_diameter):

	volume = []
	for i in xrange(len(model_names)):
		try:
			model = scipy.io.loadmat(model_names[i])
			model_volume = 0
			for j in xrange(len(model['Rad'])):
				if(model['Rad'][j][0] * 2 >= min_diameter):
					model_volume += math.pi * model['Rad'][j][0] * model['Rad'][j][0] * model['Len'][j][0]
			volume.append(model_volume)
		except:
			continue
	volume = numpy.array(volume)
	mean = numpy.mean(volume)
	stddev = numpy.std(volume)
	coeff_variation = stddev / mean
	return mean,coeff_variation

def getTrunkCylinderFromModel(model,z_coordinate):

	z = 0
	for i in xrange(len(model['CiB'][0][0])):
		z += model['Len'][(model['CiB'][0][0][i][0]-1)]
		if(z >= z_coordinate):
			cylinder = model['CiB'][0][0][i][0]
			break
	return cylinder

def getCloudFromModel(cloud,model,cylinder):

	cylinder = cylinder-1 # -1 FOR PYTHON CONVENTION
	x_t = model['Sta'][cylinder][0]+(model['Len'][cylinder][0]*model['Axe'][cylinder][0])
	y_t = model['Sta'][cylinder][1]+(model['Len'][cylinder][0]*model['Axe'][cylinder][1])
	z_t = model['Sta'][cylinder][2]+(model['Len'][cylinder][0]*model['Axe'][cylinder][2])
	x_b = model['Sta'][cylinder][0]
	y_b = model['Sta'][cylinder][1]
	z_b = model['Sta'][cylinder][2]
	x_min = min(x_t-model['Rad'][cylinder],x_b-model['Rad'][cylinder])
	x_max = max(x_t+model['Rad'][cylinder],x_b+model['Rad'][cylinder])
	y_min = min(y_t-model['Rad'][cylinder],y_b-model['Rad'][cylinder])
	y_max = max(y_t+model['Rad'][cylinder],y_b+model['Rad'][cylinder])
	z_min = min(z_t,z_b)
	z_max = max(z_t,z_b)
	out = []
	#f1 = open('test.xyz','w')
	for i in xrange(len(cloud)):
		if(cloud[i][0] >= x_min and cloud[i][0] <= x_max and cloud[i][1] >= y_min and cloud[i][1] <= y_max and cloud[i][2] >= z_min and cloud[i][2] <= z_max):
			out.append([cloud[i][0],cloud[i][1],cloud[i][2]])
			#tmp_out = str(cloud[i][0])+' '+str(cloud[i][1])+' '+str(cloud[i][2])+'\n'
			#f1.write(tmp_out)
	#f1.close()
	cloud = numpy.array(out)
	return cloud

def getDiameterFromCloud(cloud):

	try:
		dbh_pts = cloud
		x=dbh_pts[:,0]
		y=dbh_pts[:,1]
		z=dbh_pts[:,2]
		x_m = x.mean()
		y_m = y.mean()
		u = x - x_m
		v = y - y_m
		Suv = sum(u*v)
		Suu = sum(u**2)
		Svv = sum(v**2)
		Suuv = sum(u**2*v)
		Suvv = sum(u*v**2)
		Suuu = sum(u**3)
		Svvv = sum(v**3)
		A = numpy.array([[ Suu, Suv ],[Suv, Svv]])
		B = numpy.array([Suuu+Suvv,Svvv+Suuv])/2.0
		uc, vc = numpy.linalg.solve(A, B)
		xc_1 = x_m+uc
		yc_1 = y_m+vc
		Ri_1 = numpy.sqrt((x-xc_1)**2 +(y-yc_1)**2)
		r_final = numpy.mean(Ri_1)
		diameter = r_final * 2
		#residu_1 = sum((Ri_1-r_final)**2)
		return diameter,x_m,y_m
	except:
		return numpy.nan,numpy.nan,numpy.nan

def getTrunkCloudModelComparison(cloud_name,model_names):

	###
	TRUNK_POSITIONS = [0.075,0.1,0.125,0.15]
	###
	cloud = numpy.loadtxt(cloud_name)
	tree_diff = []
	for i in xrange(len(model_names)):
		#this is a very bad idea - bug in scipy.io.loadmat gives occasional error
		try:
			model = scipy.io.loadmat(model_names[i])
			trunk_length = model['BLen'][0][0]
			position_diff = []
			for j in xrange(len(TRUNK_POSITIONS)):
				cylinder = getTrunkCylinderFromModel(model,trunk_length*TRUNK_POSITIONS[j]) ### IMPO - IN MATLAB CONVENTION
				model_diameter = (model['Rad'][(cylinder-1)][0])*2 # -1 FOR PYTHON CONVENTION 
				trunk_cloud = getCloudFromModel(cloud,model,cylinder)
				cloud_diameter,cloud_x,cloud_y = getDiameterFromCloud(trunk_cloud)
				diff = min(cloud_diameter,model_diameter)/max(cloud_diameter,model_diameter)
#				diff = numpy.abs(model_diameter-cloud_diameter)/numpy.abs(cloud_diameter)
				position_diff.append(diff)
			tree_diff.append(position_diff)
		except:
			continue
	tree_diff = numpy.array(tree_diff)
	result = tree_diff[~numpy.isnan(tree_diff)].mean()
#	average_diff = []
#	for k in xrange(len(tree_diff)):
#		average_diff.append(scipy.stats.nanmean(tree_diff[k,:]))
#	average_diff = numpy.array(average_diff)
#	result = scipy.stats.nanmean(average_diff)
#	print tree_diff
#	print average_diff
#	print result
	return result

def optimise(model_id,cloud_dir,model_dir):

	cloud_search = cloud_dir+model_id+'.txt'
	cloud = glob.glob(cloud_search)[0]
	dmin_search = model_dir+model_id+"-*.mat"
	fnames = glob.glob(dmin_search)
	dmin_range = []
	for i in xrange(len(fnames)):
		dmink = float(fnames[i].split("-")[4])
		dmin_range.append(dmink)
	dmin_range = numpy.array(dmin_range)
	dmin_range = numpy.unique(dmin_range)
	dmin_range = numpy.sort(dmin_range)
	metadata = []
	for j in xrange(len(dmin_range)):
		model_search = model_dir+model_id+'-*-*-*-'+str(dmin_range[j])+'-*-*-*-*-*.mat'
		models = glob.glob(model_search)
		if(len(models) >= 3):
			mean,coeff_variation = getVolume(models,0)
			trunk_difference = getTrunkCloudModelComparison(cloud,models)
			metadata.append([dmin_range[j],mean,coeff_variation,trunk_difference])
	metadata = numpy.array(metadata)
	metadata_name = model_id + '.opt'
	numpy.savetxt(metadata_name,metadata,fmt='%.4f')
	##		
	dmin_opt = numpy.nan
	##
	min_cov = numpy.min(metadata[:,2,])
	max_conf = numpy.max(metadata[:,3])
	for m in xrange(len(metadata)):
		if(metadata[m][2] < min_cov * 2 and metadata[m][3] > max_conf * 0.95):
			dmin_opt = metadata[m][0]
			break
	if(numpy.isnan(dmin_opt) == True):
		idx = numpy.argmin(metadata[:,2])
		dmin_opt = metadata[idx][0]
	##
	model_search = model_dir+model_id+'-*-*-*-'+str(dmin_opt)+'-*-*-*-*-*.mat'
	models = glob.glob(model_search)
	results,best_model = getResults(models,0)
	numpy.savetxt(model_id+'.dat',results,fmt='%s %.3f %.3f')
	shutil.copy(best_model,model_id+'.mat')
	##would be nice if pdfcrop was ported to python
	image_name = model_id+'.pdf'
	plotTrees.plotCloudsModels([cloud],[model_id+'.mat'],0,0,image_name)
	crop_string = 'pdfcrop '+image_name+' '+image_name
	os.system(crop_string)

if __name__ == "__main__":

	def func_star(a_b):
	    return optimise(*a_b)

	parser = argparse.ArgumentParser()
	parser.add_argument('-c','--cloud_dir',default='../clouds/',help='cloud directory')
	parser.add_argument('-m','--model_dir',default='./',help='model directory')
	parser.add_argument('-p','--multiprocess',default=1,help='number of workers')
	args = parser.parse_args()
	model_fnames = glob.glob(args.model_dir+"*-*.mat")
	model_id = []
	for i in xrange(len(model_fnames)):
		mid = model_fnames[i].split('/')[len(model_fnames[i].split('/'))-1].split('-')[0]
		model_id.append(mid)
	model_id = numpy.array(model_id)
	model_id = numpy.unique(model_id)
	cloud_fnames = glob.glob(args.cloud_dir+"*_*.txt")
	cloud_id = []
	for j in xrange(len(cloud_fnames)):
		cid = cloud_fnames[j].split('/')[len(cloud_fnames[j].split('/'))-1].split('.')[0]
		cloud_id.append(cid)
	tid = list(set(cloud_id).intersection(model_id))
#	for k in xrange(len(tid)):
#		optimise(tid[k],args.cloud_dir,args.model_dir)
	pool = multiprocessing.Pool(int(args.multiprocess))	
	pool.map(func_star, itertools.izip(tid, itertools.repeat(args.cloud_dir), itertools.repeat(args.model_dir)))

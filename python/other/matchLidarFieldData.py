#!/usr/bin/env python

#Andrew Burt - a.burt.12@ucl.ac.uk

import argparse
import numpy as np

class matchLidarFieldData:

	def __init__(self):

		self.MAX_DELTA_X = 7.5
		self.MAX_DELTA_Y = 7.5
		self.MAX_DELTA_H = 10
		self.MAX_DELTA_D = 0.01

	def getLidarData(self,infile):

		#lid lx ly lh ldbh le lv lv- lv+ 
		data = np.loadtxt(infile,dtype=[('lid','S16'),('lx',float),('ly',float),('lh',float),('ldbh',float),('le',float),('lv',float),('lv_min',float),('lv_max',float)])
		return data

	def getFieldData(self,infile):

		#fid fspecies fx fy fh fdbh fpom fwd
		data = np.loadtxt(infile,dtype=[('fid','S16'),('fspecies','S16'),('fx',float),('fy',float),('fh',float),('fdbh',float),('fpom',float),('wd',float)])
		return data

	def matchData(self,lidarData,fieldData):

		matchedData = []
		complete = False
		while(complete == False):
			matrix = np.empty((len(fieldData),len(lidarData)))
			matrix[:] = np.NAN
			for i in xrange(len(fieldData)):
				for j in xrange(len(lidarData)):
					d_x = np.abs(lidarData['lx'][j]-fieldData['fx'][i])
					d_y = np.abs(lidarData['ly'][j]-fieldData['fy'][i])
					d_h = np.abs(lidarData['lh'][j]-fieldData['fh'][i])
					d_d = np.abs(lidarData['ldbh'][j]-fieldData['fdbh'][i])
					if(d_x <= self.MAX_DELTA_X and d_y <= self.MAX_DELTA_Y and d_h <= self.MAX_DELTA_H and d_d <= self.MAX_DELTA_D):
						dd_x = np.abs(lidarData['lx'][j]-fieldData['fx'][i])/np.abs(fieldData['fx'][i])
						dd_y = np.abs(lidarData['ly'][j]-fieldData['fy'][i])/np.abs(fieldData['fy'][i])
						dd_h = np.abs(lidarData['lh'][j]-fieldData['fh'][i])/np.abs(fieldData['fh'][i])
						dd_d = np.abs(lidarData['ldbh'][j]-fieldData['fdbh'][i])/np.abs(fieldData['fdbh'][i])
						total = dd_x + dd_y + dd_h + dd_d
						matrix[i][j] = total
			try:
				idx = np.unravel_index(np.nanargmin(matrix),matrix.shape)
				tmp = []
				tmp.append(fieldData['fid'][idx[0]]) #fid
				tmp.append(lidarData['lid'][idx[1]]) #lid
				tmp.append(fieldData['fspecies'][idx[0]]) #fspecies
				tmp.append(fieldData['fx'][idx[0]]) #fx
				tmp.append(fieldData['fy'][idx[0]]) #fy
				tmp.append(fieldData['fh'][idx[0]]) #fh
				tmp.append(fieldData['fdbh'][idx[0]]) #fdbh
				tmp.append(fieldData['fpom'][idx[0]]) #fpom
				tmp.append(fieldData['wd'][idx[0]]) #fwd
				tmp.append(lidarData['lx'][idx[1]]) #lx
				tmp.append(lidarData['ly'][idx[1]]) #ly
				tmp.append(lidarData['lh'][idx[1]]) #lh
				tmp.append(lidarData['ldbh'][idx[1]]) #ldbh
				tmp.append(lidarData['le'][idx[1]]) #le
				tmp.append(lidarData['lv'][idx[1]]) #lv
				tmp.append(lidarData['lv_min'][idx[1]]) #lv-
				tmp.append(lidarData['lv_max'][idx[1]]) #lv+
				matchedData.append(tmp)
				fieldData = np.delete(fieldData,idx[0])
				lidarData = np.delete(lidarData,idx[1])
			except Exception:
				complete = True
		return matchedData

	def writeResultsFile(self,matchedData,outfile='matched_data.txt'):

		f1 = open(outfile,'w')
		for i in xrange(len(matchedData)):
			for j in xrange(len(matchedData[0])):
				f1.write(str(matchedData[i][j])+' ')
			f1.write("\n")
		f1.close()

if __name__ == "__main__":

	parser = argparse.ArgumentParser()
	parser.add_argument('-l','--lidarfile',default=False,help='.xyz ascii inclouds')
	parser.add_argument('-f','--fieldfile',default=False,help='.csv results file: x,y,h,dbh,pom,wd (in metric units)')
	args = parser.parse_args()
	if(args.lidarfile == False):
		sys.exit('No input files specified. -h for usage')
	else:
		example = matchLidarFieldData()
		lidarData = example.getLidarData(args.lidarfile)
		fieldData = example.getFieldData(args.fieldfile)
		matchedData = example.matchData(lidarData,fieldData)
		example.writeResultsFile(matchedData)

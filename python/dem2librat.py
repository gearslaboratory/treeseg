#!/usr/bin/env python

#Andrew Burt - a.burt.12@ucl.ac.uk

import argparse
import numpy as np
import scipy.interpolate
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

class dem2librat:
					
	def ReadDem(self,infile):
		
		f1=open(infile,'r')
		line = f1.readline()
		tmp = line.split('\n')
		header = np.fromstring(tmp[0],sep=' ') 
		self.resolution = header[0] 
		self.xmin = header[1]
		self.xmax = header[2]
		self.ymin = header[3]
		self.ymax = header[4]
		self.data = np.loadtxt(infile,skiprows=1)

	def CreateMesh(self):

		#nx = ((self.xmax-self.xmin)/self.resolution)+1
		#ny = ((self.ymax-self.ymin)/self.resolution)+1
		#xi = np.linspace(self.xmin,self.xmax,nx)
		#yi = np.linspace(self.ymin,self.ymax,ny)
		#xi,yi = np.meshgrid(xi,yi)

		#THIS IS A TEMP FIX - NEED TO USE method=cubic but only interpolates, not extrapolates

		x_start = self.xmin + 5
		x_end = self.xmax - 5
		y_start = self.ymin + 5
		y_end = self.ymax - 5

		nx = ((self.xmax-self.xmin)/self.resolution)+5
		ny = ((self.xmax-self.xmin)/self.resolution)+5
 
		xi = np.linspace(x_start,x_end,nx)
		yi = np.linspace(y_start,y_end,ny)
		xi,yi = np.meshgrid(xi,yi)

		return xi,yi

	def InterpolateMesh(self,xi,yi):

		zi = scipy.interpolate.griddata((self.data[:,0],self.data[:,1]),self.data[:,2],(xi,yi),method='linear')
		print zi
		return zi

	def plotSurface(self,xi,yi,zi):

		fig = plt.figure()
		ax = fig.add_subplot(111, projection='3d')
		ax.plot_surface(xi,yi,zi,rstride=int(self.resolution),cstride=int(self.resolution),cmap=cm.jet,linewidth=0.5)
		plt.savefig('dem.png',dpi=150)

	def CreateObj(self,xi,yi,zi):

		facets = []
		for i in xrange(len(xi)-1):
			for j in xrange(len(yi)-1):
				tmp = []
				tmp.append('usemtl ground')
				tmp.append('v '+str(xi[i][j])+' '+str(yi[i][j])+' '+str(zi[i][j]))
				tmp.append('v '+str(xi[i+1][j])+' '+str(yi[i+1][j])+' '+str(zi[i+1][j]))
				tmp.append('v '+str(xi[i+1][j+1])+' '+str(yi[i+1][j+1])+' '+str(zi[i+1][j+1]))
				tmp.append('f -3 -2 -1')
				tmp.append('usemtl ground')
				tmp.append('v '+str(xi[i+1][j+1])+' '+str(yi[i+1][j+1])+' '+str(zi[i+1][j+1]))
				tmp.append('v '+str(xi[i][j+1])+' '+str(yi[i][j+1])+' '+str(zi[i][j+1]))
				tmp.append('v '+str(xi[i][j])+' '+str(yi[i][j])+' '+str(zi[i][j]))
				tmp.append('f -3 -2 -1')
				facets.append(tmp)
		return facets

	def WriteObj(self,facets,outfile,name):

		f1 = open(outfile,'w')
		f1.write('!{\n')
		f1.write(name+'\n')
		f1.write('#define\n')
		f1.write('!{\n')
		f1.write('!{\n')
		f1.write('usemtl ground\n')
		f1.write('!}\n')
		f1.write('!}\n')
		for i in xrange(len(facets)):
			for j in xrange(len(facets[i])):
				f1.write(facets[i][j]+'\n')
		f1.write('!}\n')
		f1.close()

if __name__ == "__main__":
	
	parser = argparse.ArgumentParser()
	parser.add_argument('-i', '--infile',default=False,help='cylinder_tree_model.m output in MATLAB .mat format')
	parser.add_argument('-o', '--outfile',default=False,help='librat outfilein .obj format')
	parser.add_argument('-n', '--name',default='g plant 0',help='Object name ie g plant 0')
	args = parser.parse_args()
	example = dem2librat()
	example.ReadDem(args.infile)
	xi,yi = example.CreateMesh()
	zi = example.InterpolateMesh(xi,yi)
	facets = example.CreateObj(xi,yi,zi)
	example.WriteObj(facets,args.outfile,args.name)
	example.plotSurface(xi,yi,zi)

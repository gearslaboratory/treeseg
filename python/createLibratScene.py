#!/usr/bin/env python

#Andrew Burt - a.burt.12@ucl.ac.uk

import argparse
from cyl2librat import cyl2librat
from dem2librat import dem2librat

class createLibratScene:

	def __init__(self,models,dem):

		self.model_names = models
		self.dem_name = dem

	def createTreeObj(self):

		for i in xrange(len(self.model_names)):
			tmp = self.model_names[i].split('.')
			fout = tmp[0] +'.obj.bbox'
			name = 'g plant '+tmp[0]
			treeObject = cyl2librat(self.model_names[i],False,fout,50,name)
			treeObject.DefineBBox()
			treeObject.CreateBBox(level=0)
			treeObject.WriteBBoxLibrat()

	def createDemObj(self):

		demObject = dem2librat()
		demObject.ReadDem(self.dem_name)
		xi,yi = demObject.CreateMesh()
		zi = demObject.InterpolateMesh(xi,yi)
		facets = demObject.CreateObj(xi,yi,zi)
		demObject.WriteObj(facets)

	def createScene(self):

		f1 = open('scene.obj','w')
		f1.write('mtllib plants.matlib\n')
		for i in xrange(len(self.model_names)):
			tmp = self.model_names[i].split('.')
			f1.write('#include '+tmp[0]+'.obj.bbox\n')
		f1.write('!{\n')
		for j in xrange(len(self.model_names)):
			tmp = self.model_names[j].split('.')
			f1.write('clone 0 0 0 0 plant '+tmp[0]+'\n')
		f1.write('!}\n')
		f2 = open('plants.matlib','w')
		f2.write('srm wood wood.dat\n')
		f2.close()
		f3 = open('wood.dat','w')
		f3.write('0 0\n')
		f3.close()			

if __name__ == "__main__":

	parser = argparse.ArgumentParser()
	parser.add_argument('-i','--infiles',nargs='*',default=False)
	parser.add_argument('-d','--dem',default=False)
	args = parser.parse_args()
	example = createLibratScene(args.infiles,args.dem)
	example.createTreeObj()
	#example.createDemObj()
	example.createScene()

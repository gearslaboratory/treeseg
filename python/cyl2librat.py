#!/usr/bin/env python

#Andrew Burt - a.burt.12@ucl.ac.uk

import sys
import argparse
import numpy as np
import scipy.io

class cyl2librat:
	
	def __init__(self,file_in,file_out,file_out_bound,fpb,name):

		self.f1 = file_in
		self.f2 = file_out
		self.f3 = file_out_bound
		self.fpb = int(fpb)
		self.name = name
		self.H = 1
		self.ReadModel()
		self.CreateObj()
				
	def ReadModel(self):

		self.data = scipy.io.loadmat(self.f1)

	def CreateObj(self):

		cylinder_count = len(self.data['Rad'])
		self.v = np.zeros((cylinder_count*2,3))
		self.facets = []
		i = 0
		c = 0
		while(c < cylinder_count):
#			if(self.data['Rad'][c][0] > 0.35):
			self.v[i][0] = self.data['Sta'][c][0]+(self.data['Len'][c][0]*self.data['Axe'][c][0])
			self.v[i][1] = self.data['Sta'][c][1]+(self.data['Len'][c][0]*self.data['Axe'][c][1])
			self.v[i][2] = self.data['Sta'][c][2]+(self.data['Len'][c][0]*self.data['Axe'][c][2])
			self.v[i+1][0] = self.data['Sta'][c][0]
			self.v[i+1][1] = self.data['Sta'][c][1]
			self.v[i+1][2] = self.data['Sta'][c][2]
			radius = self.data['Rad'][c][0]
			for j in xrange(4):
				line = ''
				if(j==0):
					line = 'usemtl wood'
				elif(j==1):
					line = 'v '+str(self.v[i][0])+' '+str(self.v[i][1])+' '+str(self.v[i][2])
				elif(j==2):
					line = 'v '+str(self.v[i+1][0])+' '+str(self.v[i+1][1])+' '+str(self.v[i+1][2])
				elif(j==3):
					line = 'ccyl -2 -1 '+str(radius)
				tmp = line.split()
				self.facets.append(tmp)
			i += 2
			c += 1 					

	def WriteStdObj(self):

		f2 = open(self.f2,'w')
		f2.write('!{\n')
		f2.write(self.name+'\n')
		f2.write('#define\n')
		f2.write('!{\n')
		f2.write('!{\n')
		f2.write('usemtl wood\n')
		f2.write('!}\n')
		f2.write('!}\n')
		i = 0
		while i < len(self.facets):
			for j in xrange(4):
				for k in xrange(len(self.facets[i+j])):
					f2.write(self.facets[i+j][k])
					f2.write(' ')
				f2.write('\n')	
			i += 4
		f2.write('!}\n')
		f2.close()

	def DefineBBox(self):

		Max = self.v.max(axis=0)
		Min = self.v.min(axis=0)	
		xMax = Max[0] * 1.01
		xMin = Min[0] * 1.01
		yMax = Max[1] * 1.01
		yMin = Min[1] * 1.01
		zMax = Max[2] * 1.01
		zMin = Min[2] * 1.01
		i = 0	
		while i < 1:
			i = 1
			pwr = 2**self.H
			self.Boxes = np.zeros((pwr+1,pwr+1,pwr+1,self.fpb))
			self.nBoxes = np.zeros((pwr+1,pwr+1,pwr+1))
			a = 0
			while a < len(self.facets):	
				xBox = max(0,min(pwr,int((float(self.facets[a+2][1])-xMin)/((1.0/pwr)*(xMax-xMin)))))
				yBox = max(0,min(pwr,int((float(self.facets[a+2][2])-yMin)/((1.0/pwr)*(yMax-yMin)))))
				zBox = max(0,min(pwr,int((float(self.facets[a+2][3])-zMin)/((1.0/pwr)*(zMax-zMin)))))
				try:
					self.Boxes[xBox][yBox][zBox][self.nBoxes[xBox][yBox][zBox]] = a
					self.nBoxes[xBox][yBox][zBox] += 1
					a += 4
				except IndexError:
					if (self.H < 7):
						self.H += 1
						i -= 1
						break
					else:
						sys.exit("H>7, increase -f")
		#print 'H = '+str(self.H)
		self.Final = []
		self.Final.append('!{')
		self.Final.append(self.name)
		self.Final.append('#define')
		self.Final.append('!{')	
		self.Final.append('!{')
		self.Final.append('usemtl wood')
		self.Final.append('!}')
		self.Final.append('!}')
		self.Level = np.zeros((self.H,3))
				
	def CreateBBox(self,level):
	
		IJK = np.zeros((3))
		for i in xrange(0,2):
			for j in xrange(0,2):
				for k in xrange(0,2):
					self.Final.append('!{')
					self.Level[level][0]=i;
					self.Level[level][1]=j;
					self.Level[level][2]=k;
					if (level==self.H-1):
						for l in xrange(0,3):
							IJK[l]=0
						for l in xrange(0,level+1):
							base=2**(level-l)
							for m in xrange(0,3):
								IJK[m]+=self.Level[l][m]*base
						self.Final.append('g box 1'+str(int(IJK[0]))+str(int(IJK[1]))+str(int(IJK[2])))
						for l in xrange(0,int(self.nBoxes[IJK[0]][IJK[1]][IJK[2]])):
							n = self.Boxes[IJK[0]][IJK[1]][IJK[2]][l]
							self.Final.append(str(self.facets[int(n)][0])+' '+str(self.facets[int(n)][1]))
							self.Final.append(str(self.facets[int(n+1)][0])+' '+str(self.facets[int(n+1)][1])+' '+str(self.facets[int(n+1)][2])+' '+str(self.facets[int(n+1)][3]))
							self.Final.append(str(self.facets[int(n+2)][0])+' '+str(self.facets[int(n+2)][1])+' '+str(self.facets[int(n+2)][2])+' '+str(self.facets[int(n+2)][3]))
							self.Final.append(str(self.facets[int(n+3)][0])+' '+str(self.facets[int(n+3)][1])+' '+str(self.facets[int(n+3)][2])+' '+str(self.facets[int(n+3)][3]))
					else:
						self.CreateBBox(level+1)
					self.Final.append('!}')

	def WriteBBoxLibrat(self):

		self.Final.append('!}')
		r__ = [0] * len(self.Final)
		p1__ = [0] * len(self.Final)
		r = []
		p1 = []
		__NR = -1
		for i in xrange(len(self.Final)):
			__NR += 1
			tmp = self.Final[i]
			tmp_s = tmp.split()
			p1__[__NR] = tmp_s[0]
			r__[__NR] = self.Final[i]
			if(p1__[__NR] == '!}' and p1__[__NR-1][0] == 'g' and p1__[__NR-2] == '!{'):
				__NR -= 3
			elif(p1__[__NR] == '!}' and p1__[__NR-1] == '!{'):
				__NR -= 2
		for i in xrange(0,__NR):
			if (r__[i] != 0):
				r.append(r__[i])
				p1.append(p1__[i])
		__NR = len(r)
		NR=__NR
		nChanges = 1
		pass1 = 0
		_p1 = [0] * __NR
		_r = [0] * __NR
		while (nChanges > 0):
			pass1 += 1
			nChanges = 0
			_NR = -1
			i = 0
			while (i < NR):
				if (p1[i] == '!{' and p1[i+1] == '!}'):
					nChanges += 1
					i += 1
				elif (p1[i] =='!{' and p1[i+1] =='B' and p1[i+2] == '!}'):
					_NR += 1
					_p1[_NR] =p1[i+1]
					_r[_NR]=r[i+1]
					i+=2
					nChanges+=1
				elif (p1[i]=="!{" and p1[i+1]=="!}"):
       					nChanges+=1
					i+=1
				elif(p1[i]=="!{" and p1[i+1]=="g" and p1[i+2]=="!}"):
					nChanges+=1
					i+=2
				elif(p1[i]=="!{" and p1[i+1]=="g" and p1[i+2]!="!{" ):
					_NR += 1
					_r[_NR] = str(r[i])+"\n"+str(r[i+1])
					_p1[_NR]="B"
					for j in xrange(i+2,NR+1):
						_r[_NR] = str(_r[_NR])+"\n"+str(r[j])
						if(p1[j]=="!}"):
							i=j
							nChanges+=1
							break
				else:
					_NR += 1
					_p1[_NR] = p1[i]
					_r[_NR] = r[i]
				i += 1
			for i in xrange(0,_NR+1):
				p1[i] = _p1[i]
				r[i] = _r[i]
			NR = _NR + 1
		f3 = open(self.f3,'w')
		for i in xrange(0,NR):
			f3.write(str(r[i])+'\n')
		f3.write('!}')
		f3.close()

if __name__ == "__main__":
	
	parser = argparse.ArgumentParser()
	parser.add_argument('-i', '--infile',default=False,help='cylinder_tree_model.m output in MATLAB .mat format')
	parser.add_argument('-o', '--outfile',default=False,help='librat outfilein .obj format')
	parser.add_argument('-b', '--outfile_bound',default=False,help='bound librat outfile in .obj.bbox format')
	parser.add_argument('-f', '--fpb',default=100,help='Maximum number of facets per bounding box (default 100)')
	parser.add_argument('-n', '--name',default='g plant 0',help='Object name ie g plant 0')
	args = parser.parse_args()
	if(args.infile == False):
		sys.exit('No input file specified. -h for usage')
	elif (args.outfile != False and args.outfile_bound != False):
		example = cyl2librat(args.infile,args.outfile,args.outfile_bound,args.fpb,args.name)
		example.WriteStdObj()
		example.DefineBBox()
		example.CreateBBox(level=0)
		example.WriteBBoxLibrat()
	elif (args.outfile != False and args.outfile_bound == False):
		example = cyl2librat(args.infile,args.outfile,args.outfile_bound,args.fpb,args.name)
		example.WriteStdObj()
	elif (args.outfile == False and args.outfile_bound != False):
		example = cyl2librat(args.infile,args.outfile,args.outfile_bound,args.fpb,args.name)
		example.DefineBBox()
		example.CreateBBox(level=0)
		example.WriteBBoxLibrat()

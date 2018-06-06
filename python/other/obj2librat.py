#!/usr/bin/env python

#Andrew Burt - a.burt.12@ucl.ac.uk

import sys
import argparse
import numpy as np

class Obj2Librat:
	
	def __init__(self,file_in,file_out,file_out_bound,fpb,name):

		self.H = 1
		self.f1 = file_in
		self.fpb = int(fpb)
		self.name = name

		if (file_out != False and file_out_bound != False):

			self.f2 = file_out
			self.f3 = file_out_bound
			self.ReadObj()
			self.WriteStdLibrat()
			self.DefineBBox()
			self.CreateBBox(level=0)
			self.WriteBBoxLibrat()

		elif (file_out != False and file_out_bound == False):
			
			self.f2 = file_out
			self.ReadObj()
			self.WriteStdLibrat()

		elif (file_out == False and file_out_bound != False):

			self.f3 = file_out_bound
			self.ReadObj()
			self.DefineBBox()
			self.CreateBBox(level=0)
			self.WriteBBoxLibrat()
				
	def ReadObj(self):

		vertex_count = 1
		uv_count = 1
		facet_count = 0
		self.m = []	
		f1 = open(self.f1,'r')
		filein = f1.readlines()

		for i in xrange(len(filein)):
			if (filein[i][0] == 'v') and (filein[i][1] == ' '):
				vertex_count += 1			
			elif (filein[i][0] == 'v') and (filein[i][1] == 't'):
				uv_count += 1
			elif (filein[i][0] == 'f') and (filein[i][1] == ' '):
				facet_count += 1
			elif (filein[i][0] == 'u') and (filein[i][1] == 's'):
				tmpline = filein[i].split('\n')
				self.m.append(tmpline[0])

		f1.close()
		self.v = np.zeros((vertex_count,4))
		vt = np.zeros((uv_count,4))
		f = np.zeros((facet_count,6))
		mc = np.zeros((len(f),4))
		vertex_count = 1
		uv_count = 1
		facet_count = 0
		tex_num = -1
		
		for j in xrange(len(filein)):
			tmpline = filein[j].split()
			if (filein[j][0] == 'v') and (filein[j][1] == ' '):
				self.v[vertex_count][0] = tmpline[1]
				###WHY THIS IS ODD???###
				self.v[vertex_count][1] = tmpline[3]
				self.v[vertex_count][2] = tmpline[2]
				vertex_count += 1
			if (filein[j][0] == 'v') and (filein[j][1] == 't'):
				vt[uv_count][0] = -1
				vt[uv_count][1] = tmpline[1]
				vt[uv_count][2] = tmpline[2]
				uv_count += 1
			if (filein[j][0] == 'f') and (filein[j][1] == ' '):
				tmpf1 = tmpline[1].split('/')
				tmpf2 = tmpline[2].split('/')
				tmpf3 = tmpline[3].split('/')
				f[facet_count][0] = tmpf1[0]
				f[facet_count][1] = tmpf1[1]
				f[facet_count][2] = tmpf2[0]
				f[facet_count][3] = tmpf2[1]
				f[facet_count][4] = tmpf3[0]
				f[facet_count][5] = tmpf3[1]
				mc[facet_count][0] = tex_num
				facet_count += 1
			if (filein[j][0] == 'u') and (filein[j][1] == 's'):
				tex_num += 1

		self.facets = np.zeros(((len(f)*7,4)))
		face_count = 0
		i = 0
		while i < len(self.facets):
			self.facets[i] = mc[face_count]
			self.facets[i+1] = self.v[f[face_count][0]]
			self.facets[i+2] = vt[f[face_count][1]]
			self.facets[i+3] = self.v[f[face_count][2]]
			self.facets[i+4] = vt[f[face_count][3]]
			self.facets[i+5] = self.v[f[face_count][4]]
			self.facets[i+6] = vt[f[face_count][5]]
			face_count += 1
			i += 7

	def WriteStdLibrat(self):

		f2 = open(self.f2,'w')
		f2.write('!{\n')
		f2.write(self.name+'\n')
		f2.write('#define\n')
		f2.write('!{\n')
		for i in xrange(len(self.m)):		
			f2.write('!{\n')
			f2.write(self.m[i]+'\n')
			f2.write('!}\n')
		f2.write('!}\n')
		i = 0
		while i < len(self.facets):
			#f2.write('x 7 '+self.m[int(self.facets[i][0])]+'\n')
			f2.write(self.m[int(self.facets[i][0])]+'\n')
			f2.write('v '+str(self.facets[i+1][0])+' '+str(self.facets[i+1][1])+' '+str(self.facets[i+1][2])+'\n')
			f2.write('!local '+str(self.facets[i+2][0])+' '+str(self.facets[i+2][1])+' '+str(self.facets[i+2][2])+'\n')
			f2.write('v '+str(self.facets[i+3][0])+' '+str(self.facets[i+3][1])+' '+str(self.facets[i+3][2])+'\n')
			f2.write('!local '+str(self.facets[i+4][0])+' '+str(self.facets[i+4][1])+' '+str(self.facets[i+4][2])+'\n')
			f2.write('v '+str(self.facets[i+5][0])+' '+str(self.facets[i+5][1])+' '+str(self.facets[i+5][2])+'\n')
			f2.write('!local '+str(self.facets[i+6][0])+' '+str(self.facets[i+6][1])+' '+str(self.facets[i+6][2])+'\n')
			f2.write('f -3 -2 -1\n')	
			i +=7
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

		#print xMax,xMin,yMax,yMin,zMax,zMin

		i = 0	
		while i < 1:
			i = 1
			pwr = 2**self.H
			self.Boxes = np.zeros((pwr,pwr,pwr,self.fpb))
			self.nBoxes = np.zeros((pwr,pwr,pwr))
			a = 0
			while a < len(self.facets):	
				xBox = max(0,min(pwr,int((self.facets[a+1][0]-xMin)/((1.0/pwr)*(xMax-xMin)))))
				yBox = max(0,min(pwr,int((self.facets[a+1][1]-yMin)/((1.0/pwr)*(yMax-yMin)))))
				zBox = max(0,min(pwr,int((self.facets[a+1][2]-zMin)/((1.0/pwr)*(zMax-zMin)))))
				try:
					self.Boxes[xBox][yBox][zBox][self.nBoxes[xBox][yBox][zBox]] = a
					self.nBoxes[xBox][yBox][zBox] += 1
					a += 7
				except IndexError:
					if (self.H < 7):
						self.H += 1
						i -= 1
						break
					else:
						sys.exit("H>7, increase -f")

		print 'H = '+str(self.H)

		self.Final = []
		self.Final.append('!{')
		self.Final.append(self.name)
		self.Final.append('#define')
		self.Final.append('!{')
		for i in xrange(len(self.m)):		
			self.Final.append('!{')
			self.Final.append(self.m[i])
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
							self.Final.append(self.m[int(self.facets[n][0])])
							self.Final.append('v '+str(self.facets[n+1][0])+' '+str(self.facets[n+1][1])+' '+str(self.facets[n+1][2]))
							self.Final.append('!local '+str(self.facets[n+2][0])+' '+str(self.facets[n+2][1])+' '+str(self.facets[n+2][2]))
							self.Final.append('v '+str(self.facets[n+3][0])+' '+str(self.facets[n+3][1])+' '+str(self.facets[n+3][2]))
							self.Final.append('!local '+str(self.facets[n+4][0])+' '+str(self.facets[n+4][1])+' '+str(self.facets[n+4][2]))
							self.Final.append('v '+str(self.facets[n+5][0])+' '+str(self.facets[n+5][1])+' '+str(self.facets[n+5][2]))
							self.Final.append('!local '+str(self.facets[n+6][0])+' '+str(self.facets[n+6][1])+' '+str(self.facets[n+6][2]))
							self.Final.append('f -3 -2 -1')	
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
	parser.add_argument('-i', '--infile',default=False,help='Wavefront object infile')
	parser.add_argument('-o', '--outfile',default=False,help='Librat outfile')
	parser.add_argument('-b', '--outfile_bound',default=False,help='Librat bound outfile')
	parser.add_argument('-f', '--fpb',default=100,help='Maximum number of facets per box (default 100)')
	parser.add_argument('-n', '--name',default='g plant 0',help='Object name')
	args = parser.parse_args()

	if (args.infile == False):
		sys.exit('No input file. -h for usage')
	elif (args.outfile == False and args.outfile_bound == False):
		sys.exit('No output file. -h for usage')
	else:
		Obj2Librat(args.infile,args.outfile,args.outfile_bound,args.fpb,args.name)

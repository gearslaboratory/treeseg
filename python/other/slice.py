#!/usr/bin/env python

import sys
import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('downsample_0.xyz')
plt.figure()
ax = plt.axes()
circle = plt.Circle((55.6114,43.8088),0.494794,linestyle='dashed',linewidth=1,fill=False,hatch='/',label=r'r = 0.494, $\sigma$ = 0.035')
ax.add_patch(circle)
plt.scatter(data[:,0],data[:,1],color='k',s=1)
ax.set_ylim([43,44.5])
ax.set_xlim([54.85,56.35])
ax.set_aspect('equal')
legend = plt.legend()
legend.draw_frame(False)
plt.savefig('circle_0.png',dpi=150)

data = np.loadtxt('downsample_1.xyz')
plt.figure()
ax = plt.axes()
circle = plt.Circle((55.6138,43.7945),0.5015821,linestyle='dashed',linewidth=1,fill=False,hatch='/',label=r'r = 0.502, $\sigma$ = 0.032')
ax.add_patch(circle)
plt.scatter(data[:,0],data[:,1],color='k',s=1)
ax.set_ylim([43,44.5])
ax.set_xlim([54.85,56.35])
ax.set_aspect('equal')
legend = plt.legend()
legend.draw_frame(False)
plt.savefig('circle_1.png',dpi=150)

data = np.loadtxt('downsample_2.xyz')
plt.figure()
ax = plt.axes()
circle = plt.Circle((55.6279,43.7331),0.543802,linestyle='dashed',linewidth=1,fill=False,hatch='/',label=r'r = 0.543, $\sigma$ = 0.025')
ax.add_patch(circle)
plt.scatter(data[:,0],data[:,1],color='k',s=1)
ax.set_ylim([43,44.5])
ax.set_xlim([54.85,56.35])
ax.set_aspect('equal')
legend = plt.legend()
legend.draw_frame(False)
plt.savefig('circle_2.png',dpi=150)

data = np.loadtxt('downsample_3.xyz')
plt.figure()
ax = plt.axes()
circle = plt.Circle((55.7019,43.8384),0.425063,linestyle='dashed',linewidth=1,fill=False,hatch='/',label=r'r = 0.425, $\sigma$ = 0.026')
ax.add_patch(circle)
plt.scatter(data[:,0],data[:,1],color='k',s=1)
ax.set_ylim([43,44.5])
ax.set_xlim([54.85,56.35])
ax.set_aspect('equal')
legend = plt.legend()
legend.draw_frame(False)
plt.savefig('circle_3.png',dpi=150)

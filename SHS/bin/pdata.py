#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Date    : 2018-11-22 12:55:39
# @Author  : Your Name (you@example.org)
# @Link    : http://example.org
# @Version : $Id$

import os
import pandas as np
import numpy as np
import matplotlib.pylab as plt
import seaborn as sns

data1=np.loadtxt('../data/testp.csv',delimiter=',')

# data1=np.loadtxt('../data/data.csv',delimiter=',')
# data2=np.loadtxt("../data/1.csv",delimiter=',')
# data3=np.loadtxt("../data/5.csv",delimiter=',')
# data4=np.loadtxt("../data/13.csv",delimiter=',')
# data5=np.loadtxt("../data/4.csv",delimiter=',')
# data6=np.loadtxt("../data/2.csv",delimiter=',')



# datareal=np.loadtxt("../data/xx4.csv",delimiter=',')
# datacp=np.loadtxt("../data/pxu.csv",delimiter=',')
plt.figure(figsize=(10,10))

# plt.plot(data2[:,0],data2[:,1],marker='.',label='1')
# plt.plot(data3[:,0],data3[:,1],marker='.',label='5')
# plt.plot(data4[:,0],data4[:,1],marker='.',label='13')
# plt.plot(data5[:,0],data5[:,1],marker='o',label='4')
# plt.plot(data6[:,0],data6[:,1],marker='o',label='2')
# plt.plot(datareal[:,0],datareal[:,1],marker='*',markersize=10,label='real')
# plt.plot(datacp[:,0],datacp[:,1],marker='.',label='xu')
plt.plot(data1[:,0],data1[:,1],marker='+',markersize=1, linestyle=None,label='data',lw=0.05)
plt.grid()
plt.legend()
# plt.axis('equal')
plt.show()
#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 
# @Date    : 2018-11-29 18:02:34
# @Author  : Your Name (you@example.org)
# @Link    : http://example.org
# @Version : $Id$

import pandas as pd
import numpy as np
import matplotlib.pylab as plt
datafile='/home/aaron/projects/SHS/data/logfile_2018_11_28_16_21_23.txt'
alpha=0.85
ACCE=[]
GRAVITY=[[0,0,9.806]]
GYRO=[]
MAGN=[]
PRES=[]
AHRS=[]
WIFI=[]
POSI=[]
INDEX=[]
WIFI_list=[]
name_list=[]
with open(datafile,"r") as f:    #设置文件对象
    for index,line in enumerate(f):
        if index>35:
            comment=line[:-1].split(';') 
            if comment[0]=='ACCE':
                gx=alpha * GRAVITY[-1][0] + (1 - alpha) *float(comment[3])
                gy=alpha * GRAVITY[-1][1] + (1 - alpha) *float(comment[4])
                gz=alpha * GRAVITY[-1][2] + (1 - alpha) *float(comment[5])
                lx=float(comment[3])-gx
                ly=float(comment[4])-gy
                lz=float(comment[5])-gz
                ACCE.append([comment[1],comment[2],lx,ly,lz])
                GRAVITY.append([gx,gy,gz])
            elif comment[0]=='GYRO':
                GYRO.append(comment[1:-1])
            elif comment[0]=='MAGN':
                MAGN.append(comment[1:-1])
            elif comment[0]=='PRES':
                PRES.append(comment[1:-1])
            elif comment[0]=='AHRS':
                AHRS.append(comment[1:-1])
            elif comment[0]=='WIFI':
                WIFI.append(comment[1:])
                if comment[4] not in WIFI_list:
                    WIFI_list.append(comment[4])
                    name_list.append(comment[3])
            elif comment[0]=='POSI':
                POSI.append(comment[1:-1])
                INDEX.append(index-36)
            else:
                continue
for item in WIFI:
    item[3]=WIFI_list.index(item[3])
    item[2]=0

gam=[]
for item in range(len(MAGN)-1):
    tmp=[]
    if item in INDEX:
        tmp.append(INDEX.index(item))
    else:
        tmp.append(0)    
    tmp.extend(ACCE[item][2:])
    tmp.extend(GRAVITY[item+1][:])
    tmp.extend(MAGN[item][2:])
    tmp.extend(GYRO[item][2:])  
    tmp.extend(ACCE[item][:2])
    gam.append(tmp)
gam=np.array(gam).astype('float')

np.savetxt(datafile.replace('txt','csv'),gam,delimiter=',',fmt='%.5f')

#!/usr/bin/env python
# -*- coding: utf-8 -*-
# NIST getSensorData的处理软件
# @Date    : 2018-11-29 18:02:34
# @Author  : Your Name (you@example.org)
# @Link    : http://example.org
# @Version : $Id$

import pandas as pd
import numpy as np
# import matplotlib.pylab as plt
datafile='/media/aaron/新加卷1/工作记录/项目产出及记录/19年羲和后台过程归档/第二阶段材料整理/数据/Pixel XL(XH-MP-010)/F7_1.txt'
alpha=0.98
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
            	if len(ACCE)<0:  #如果使用自己起点的做的话，会导致收敛的慢，一开始那段偏航角会歪
            		GRAVITY=[[float(comment[3]),float(comment[4]),float(comment[5])]]
            		ACCE.append([comment[1],comment[2],0,0,0])
            	else:
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

# cout=min(len(GYRO),len(MAGN),len(ACCE))
cout=len(ACCE)
acce=np.array(ACCE).astype('float')
magnetic=np.array(MAGN).astype('float')
gyro=np.array(GYRO).astype('float')

for item in range(cout-1):
    tmp=[]
    if item in INDEX:
        tmp.append(INDEX.index(item))
    else:
        tmp.append(0)    
    tmp.extend(ACCE[item][2:])
    tmp.extend(GRAVITY[item+1][:])
    nindex=np.argmin(np.abs(magnetic[:,0]-acce[item,0]))
    tmp.extend(MAGN[nindex][2:])

    nindex=np.argmin(np.abs(gyro[:,0]-acce[item,0]))
    tmp.extend(GYRO[nindex][2:])    


    tmp.extend(ACCE[item][:2])
    gam.append(tmp)
gam=np.array(gam).astype('float')

np.savetxt(datafile.replace('txt','csv'),gam,delimiter=',',fmt='%.5f')

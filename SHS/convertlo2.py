#!/usr/bin/env python
# -*- coding: utf-8 -*-
#SENSOR LOG的处理软件
# @Date    : 2018-11-29 18:02:34
# @Author  : Your Name (you@example.org)
# @Link    : http://example.org
# @Version : $Id$

import pandas as pd
import numpy as np
import os
# import matplotlib.pylab as plt
alldatadir='/media/aaron/新加卷/工作记录/项目产出及记录/IPIN19/IPIN2019/logfiles2019/Logfiles/01-Training/01a-Regular'
for item in os.listdir(alldatadir):
    filedir=os.path.join(alldatadir,item)
    if not os.path.isdir(filedir):
        continue
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


    with open(os.path.join(filedir,'gyroscope-calibrated.txt')) as target:
        for index,line in enumerate(target):
            if index>6:
                comment=line[:-1].split(' ')
                GYRO.append(comment) 

    with open(os.path.join(filedir,'magnetometer-calibrated.txt')) as target:
        for index,line in enumerate(target):
            if index>6:
                comment=line[:-1].split(' ')
                MAGN.append(comment) 
    # with open(os.path.join(filedir,'pressure.txt')) as target:
    #     for index,line in enumerate(target):
    #         if index>6:
    #             comment=line[:-1].split(' ')
    #             PRES.append(comment)

    with open(os.path.join(filedir,'accelerometer.txt')) as target:
        for index,line in enumerate(target):
            if index>6:
                comment=line[:-1].split(' ') 
                if len(ACCE)<=0:
                    GRAVITY=[[float(comment[2]),float(comment[3]),float(comment[4])]]
                    ACCE.append([comment[0],comment[1],0,0,0])
                else:
                    gx=alpha * GRAVITY[-1][0] + (1 - alpha) *float(comment[2])
                    gy=alpha * GRAVITY[-1][1] + (1 - alpha) *float(comment[3])
                    gz=alpha * GRAVITY[-1][2] + (1 - alpha) *float(comment[4])              
                    lx=float(comment[2])-gx
                    ly=float(comment[3])-gy
                    lz=float(comment[4])-gz
                    ACCE.append([comment[0],comment[1],lx,ly,lz])
                    GRAVITY.append([gx,gy,gz]) 


                            




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

    fname=filedir.split('/')[-1]
    np.savetxt(os.path.join('/media/aaron/新加卷/工作记录/项目产出及记录/IPIN19/IPIN2019/logfiles2019/Logfiles/test','%s.csv'%fname),gam,delimiter=',',fmt='%.5f')

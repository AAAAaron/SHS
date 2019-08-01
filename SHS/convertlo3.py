#!/usr/bin/env python
# -*- coding: utf-8 -*-
#批量处理getsensor的数据
# @Date    : 2018-11-29 18:02:34
# @Author  : Your Name (you@example.org)
# @Link    : http://example.org
# @Version : $Id$

import pandas as pd
import numpy as np
import os
import sqlite3
# conn = sqlite3.connect('/home/aaron/project/Object-Detection-Learn/test.db')
# c = conn.cursor()
# import matplotlib.pylab as plt
data_offset=38
dbIndex=0
alldatadir='/media/aaron/新加卷/工作记录/项目产出及记录/IPIN19/IPIN2019/logfiles2019/Logfiles/01-Training/ts'
for item in os.listdir(alldatadir):
    filedir=os.path.join(alldatadir,item)
    if  os.path.isdir(filedir):
        continue
    print(filedir)

    filename=filedir.split('/')[-1][:-4]
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
    with open(filedir,"r") as f:    #设置文件对象
        for index,line in enumerate(f):
            if index>data_offset :
                comment=line[:-1].split(';')
                if len(comment)<5:
                    continue 
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
                    # print(comment[1:])
                    POSI.append(comment[1:])
                    # c.execute("""INSERT INTO POSI (ID,FNAME,AppTimestamp,Counter,Latitude,Longitude,floorId,BuildingId)  VALUES (%d,"%s",%f, %d, %f, %f, %d, %d)""" \
                    #     %(dbIndex,filename,float(comment[1]),int(comment[2]),float(comment[3]),float(comment[4]),int(comment[5]),int(comment[6])))
                    # print("""INSERT INTO POSI (ID,FNAME,AppTimestamp,Counter,Latitude,Longitude,floorId,BuildingId)  VALUES (%d,"%s",%f, %d, %f, %f, %d, %d)""" \
                    #     %(dbIndex,filename,float(comment[1]),int(comment[2]),float(comment[3]),float(comment[4]),int(comment[5]),int(comment[6])))
                    dbIndex+=1
                    INDEX.append(index-data_offset)
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
    index_count=0
    for item in range(cout-1):
        tmp=[]
        if index_count<len(POSI) and acce[item,0]-float(POSI[index_count][0])>0:
            index_count+=1

        tmp.append(index_count-1)    
        tmp.extend(ACCE[item][2:])
        tmp.extend(GRAVITY[item+1][:])
        nindex=np.argmin(np.abs(magnetic[:,0]-acce[item,0]))
        tmp.extend(MAGN[nindex][2:])

        nindex=np.argmin(np.abs(gyro[:,0]-acce[item,0]))
        tmp.extend(GYRO[nindex][2:])    


        tmp.extend(ACCE[item][:2])
        gam.append(tmp)
    gam=np.array(gam).astype('float')
    idata=gam[gam[:,0]==-1,:]
    cali_acc=idata[idata[:,-2]>40,:]
    gam[:,10:13]-=cali_acc[:,10:13].mean(axis=0)

    
    np.savetxt(os.path.join('/media/aaron/新加卷/工作记录/项目产出及记录/IPIN19/IPIN2019/logfiles2019/Logfiles/test2','%s.csv'%filename),gam,delimiter=',',fmt='%.5f')
# conn.commit()
# conn.close()
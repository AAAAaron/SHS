/*
 * Copyright 2019 <copyright holder> <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 * this file was made by tianxiaochun.
 * if any question ,touch me by 13051526769@163.com
 * 2019,03,06
 */

#ifndef KTRANSFCOORDINATE_H
#define KTRANSFCOORDINATE_H
#include <math.h>  
#define     PI	3.141592653589793238462  

class KTransfCoordinate
{
public:
KTransfCoordinate();
~KTransfCoordinate();
void posupdate_LLH(double llh[3], double deltenu[3]);
	// 位置更新转到WGS84坐标系表示，输入经纬高和ENU坐标系变化量，输出新的经纬高
void enu2llh(double enu[3], double orgllh[3], double llh[3]);
	//  ENU坐标系转换至WGS84坐标系，输入ENU坐标x,y,z和参考点经纬高，输出WGS84坐标

void llh2enu(double llh[3], double orgllh[3], double enu[3]);
	//  WGS84坐标系转换至ENU坐标系，输入WGS84坐标经纬高和参考点经纬高，输出ENU坐标
double cal_distance_2llh(double llh_start[3],double llh_end[3]);	
	// 输入两个wgs84坐标，输出精度为0.01m的两个点
private:


void llh2xyz(double llh[3], double xyz[3]);
	//  WGS84坐标系转换至ECEF坐标系，输入经纬高，输出ECEF坐标x,y,z

void xyz2llh(double xyz[3], double llh[3]);
	//  ECEF坐标系转换至WGS84坐标系，输入ECEF坐标x,y,z,输出经纬高

void enu2xyz(double enu[3], double orgllh[3], double xyz[3]);
	//  ENU坐标系转换至ECEF坐标系，输入ENU坐标和参考点经纬高,输出ECEF坐标系x,y,z

void xyz2enu(double xyz[3], double orgllh[3], double enu[3]);
	//  ECEF坐标系转换至ENU坐标系，输入ECEF坐标x,y,z和参考点经纬高，输出ENU坐标


};

#endif // KTRANSFCOORDINATE_H

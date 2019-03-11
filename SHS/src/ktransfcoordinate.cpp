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

#include "ktransfcoordinate.h"
// 长半径：a=6378137±2（m）；
// 地球引力和地球质量的乘积：GM=3986005×108m3s-2±0.6×108m3s-2；
// 正常化二阶带谐系数：C20=-484.16685×10-6±1.3×10-9；
// 地球重力场二阶带球谐系数：J2=108263×10-8
// 地球自转角速度：ω=7292115×10-11rads-1±0.150×10-11rads-1
// 扁率f=0.003352810664
double a = 6378137.0;
double f = 1.0 / 298.257;	// 
double b = (1 - f)*a;                                  // 
double e = sqrt(a*a - b * b) / a;
double e2 = e * e;
double mean_R=(a+b)/2.0;
KTransfCoordinate::KTransfCoordinate()
{

}

KTransfCoordinate::~KTransfCoordinate()
{

}
void KTransfCoordinate::enu2llh(double enu[3], double orgllh[3], double llh[3])
{
	double neworgllh[3] = { 0,0,0 };
	double xyz[3] = { 0,0,0 };
	for(int index=0;index<2;index++)
	{
	  neworgllh[index]=orgllh[index]*PI/180.0f;
	}
	
	enu2xyz(enu, neworgllh, xyz);

	xyz2llh(xyz, llh);
	for(int index=0;index<2;index++)
	{
	  llh[index]=llh[index]*180.0f/PI;
	}
}

void KTransfCoordinate::enu2xyz(double enu[3], double orgllh[3], double xyz[3])
{
	double lat = orgllh[0], lon = orgllh[1], height = orgllh[2];
	double slat = sin(lat), clat = cos(lat), slon = sin(lon), clon = cos(lon);
	double orgxyz[3], difxyz[3];

	double R_1[3][3] = { -slon,-slat * clon,clat*clon,clon,-slat * slon,clat*slon,0,clat,slat };
	llh2xyz(orgllh, orgxyz);

	int i, j;

	for (i = 0; i<3; i++)
	{
		difxyz[i] = 0;
		for (j = 0; j<3; j++)
		{
			difxyz[i] = difxyz[i] + R_1[i][j] * enu[j];
		}
		xyz[i] = orgxyz[i] + difxyz[i];
	}
}
void KTransfCoordinate::llh2enu(double llh[3], double orgllh[3], double enu[3])
{
	double newllh[3] = { 0,0,0 };
	double neworgllh[3] = { 0,0,0 };
	double xyz[3] = { 0,0,0 };
	for(int index=0;index<2;index++)
	{
	  newllh[index]=llh[index]*PI/180.0f;
	  neworgllh[index]=orgllh[index]*PI/180.0f;
	}
	
	llh2xyz(newllh, xyz);

	xyz2enu(xyz, neworgllh, enu);

	
}
void KTransfCoordinate::llh2xyz(double llh[3], double xyz[3])
{
	double lat = llh[0], lon = llh[1], height = llh[2];

	double slat = sin(lat), clat = cos(lat), slon = sin(lon), clon = cos(lon);
	double t2lat = (tan(lat))*(tan(lat));
	double tmp = 1 - e * e;
	double tmpden = sqrt(1 + tmp * t2lat);
	double tmp2 = sqrt(1 - e * e*slat*slat);

	double x = (a*clon) / tmpden + height * clon*clat;
	double y = (a*slon) / tmpden + height * slon*slat;
	double z = (a*tmp*slat) / tmp2 + height * slat;

	xyz[0] = x;
	xyz[1] = y;
	xyz[2] = z;
}
void KTransfCoordinate::posupdate_LLH(double llh[3], double deltenu[3])
{
	for(int index=0;index<2;index++)
	{
	  llh[index]=llh[index]*PI/180.0f;
	}
	double lat = llh[0], lon = llh[1], height = llh[2];
	double DE = deltenu[0], DN = deltenu[1], DU = deltenu[2];
	double slat = sin(lat), clat = cos(lat), tlat = slat / clat;

	double sq = 1 - e2 * slat*slat, sq2 = sqrt(sq);
	double RMh = a * (1 - e2) / sq / sq2 + height; // 子午圈曲率半径
	double RNh = a / sq2 + height;              //卯酉圈曲率半径
	double clRNh = clat * RNh;

	lat += DN / RMh;
	lon += DE / clRNh;
	height += DU;

	llh[0] = lat;
	llh[1] = lon;
	llh[2] = height;
	for(int index=0;index<2;index++)
	{
	  llh[index]=llh[index]*180.0f/PI;
	}
}
void KTransfCoordinate::xyz2enu(double xyz[3], double orgllh[3], double enu[3])
{
	double lat = orgllh[0], lon = orgllh[1], height = orgllh[2];
	double slat = sin(lat), clat = cos(lat), slon = sin(lon), clon = cos(lon);
	double tmpxyz[3], orgxyz[3],tmporg[3], difxyz[3];
	enu[0] = 0;  enu[1] = 0; enu[2] = 0;

	llh2xyz(orgllh, orgxyz);

	int i;
	for (i = 0; i<3; i++)
	{
		tmpxyz[i] = xyz[i];
		tmporg[i] = orgxyz[i];
		difxyz[i] = tmpxyz[i] - tmporg[i];
	}

	double R[3][3] = { { -slon,clon,0 },{ -slat * clon,-slat * slon,clat },{ clat*clon,clat*slon,slat } };

	for (i = 0; i<3; i++)
	{
		enu[0] = enu[0] + R[0][i] * difxyz[i];
		enu[1] = enu[1] + R[1][i] * difxyz[i];
		enu[2] = enu[2] + R[2][i] * difxyz[i];
	}
}
void KTransfCoordinate::xyz2llh(double xyz[3], double llh[3])
{
	double x = xyz[0], y = xyz[1], z = xyz[2];
	double x2 = x * x, y2 = y * y, z2 = z * z;

	double b2 = b * b;
	double ep = e * (a / b);
	double r = sqrt(x2 + y2);
	double r2 = r * r;
	double E2 = a * a - b * b;
	double F = 54 * b2*z2;
	double G = r2 + (1 - e2)*z2 - e2 * E2;
	double c = (e2*e2*F*r2) / (G*G*G);
	double s = pow(double(1 + c + sqrt(c*c + 2 * c)), double(1.0 / 3.0));
	double P = F / (3 * (s + 1 / s + 1)*(s + 1 / s + 1) * G*G);
	double Q = sqrt(1 + 2 * e2*e2*P);
	double ro = -(P*e2*r) / (1 + Q) + sqrt((a*a / 2)*(1 + 1 / Q) - (P*(1 - e2)*z2) / (Q*(1 + Q)) - P * r2 / 2);
	double tmp = (r - e2 * ro)*(r - e2 * ro);
	double U = sqrt(tmp + z2);
	double V = sqrt(tmp + (1 - e2)*z2);
	double zo = (b2*z) / (a*V);

	double height = U * (a*V - b2) / (a*V);
	double lat = atan((z + ep * ep*zo) / r);
	double longth;
	double temp = atan(y / x);

	if (x >= 0)
	{
		longth = temp;
	}
	else if ((x < 0) & (y >= 0))
	{
		longth = PI + temp;
	}
	else
	{
		longth = temp - PI;
	}

	llh[0] = lat;
	llh[1] = longth;
	llh[2] = height;
}
double KTransfCoordinate::cal_distance_2llh(double llh_start[3], double llh_end[3])
{
  	double newllh_start[3] = { 0,0,0 };
	double newllh_end[3] = { 0,0,0 };
	for(int index=0;index<2;index++)
	{
		newllh_start[index]=llh_start[index]*PI/180.0d;
		newllh_end[index]=llh_end[index]*PI/180.0d;
	}
	double vlat = fabs (newllh_start[0]-newllh_end[0]);
	double vlong = fabs(newllh_start[1]-newllh_end[1]);
	double vh = fabs(newllh_start[2]-newllh_end[2]);

	double distance =  asin (sqrt(sin(vlat/2)*sin(vlat/2)+cos(newllh_start[0])*cos(newllh_end[0])*sin(vlong/2)*sin(vlong/2)));
	distance = 2.0*distance*mean_R;
	return distance;
}


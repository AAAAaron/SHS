/*
 * Copyright 2018 <copyright holder> <email>
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
 * 2018,11,16
 */

#include "gyro.h"
namespace SHS {
Gyro::Gyro() 
{
myindex=1;
// cout<<"u chose the gyro and myindex is "<<myindex<<endl;
}

Gyro::~Gyro() 
{

}
void Gyro::filterInit(double acc_x, double acc_y, double acc_z, double mag_x, double mag_y, double mag_z)
{

    	if ((acc_x==0 &&acc_y==0&&acc_z==0)||(mag_x==0 &&mag_y==0&&mag_z==0))
	{
		return;
	}

	this->Att<< 0, 0, 0 ; // XYZ顺序，即roll pitch yaw顺序,设置成当前姿态


	//初始化四元数
	if (this->IFSETYAW)
	{
		this->Att(2)=this->yaw;
		this->IFSETYAW=false;
	}

	this->quaternion=angle2quat(Att(2),Att(1),Att(0));
	this->INIT_COMPLETE=true;
}

void Gyro::filterUpdate(double gyro_x, double gyro_y, double gyro_z, double acc_x, double acc_y, double acc_z, double mag_x, double mag_y, double mag_z, double deta_time)
{
//   cout<<"u chose the gyro and myindex is "<<myindex<<endl;
  if ((acc_x==0 &&acc_y==0&&acc_z==0)||(mag_x==0 &&mag_y==0&&mag_z==0)||(gyro_x==0 &&gyro_y==0&&gyro_z==0))
	{
		return;
	}
	if (!this->INIT_COMPLETE)
	{
		this->filterInit(acc_x,acc_y,acc_z,mag_x ,mag_y,mag_z);
		return;
	}
	gyro<<gyro_x,gyro_y,gyro_z;
	gyro=gyro*conv;
	acc<<acc_x,acc_y,acc_z;
	acc = acc*conv;
	mag<<mag_x,mag_y,mag_z;
	mag = mag*conv;

	Angle_bn_b=gyro*deta_time;
	double magn = Angle_bn_b.norm();
	double angletransform;
	if (magn>1e-40)
	{
		angletransform= (sin(magn/2.0))/magn;
	}
	else
	{
		angletransform=0.5;
	}
	double ac=cos(magn/2);
	Eigen::Quaterniond q_r;
	q_r=Eigen::Quaterniond(ac,angletransform*Angle_bn_b(0),angletransform*Angle_bn_b(1),angletransform*Angle_bn_b(2));
	this->quaternion *=q_r;

	this->quaternion.normalize();
	//[yaw, pitch,roll]=quat2angle(q,'ZYX')按照roll.pitch,yaw显示
	this->Att=quat2angle(this->quaternion);
	
	// 设置yaw角度
	if (this->IFSETYAW)
	{
		this->Att(2)=this->yaw;
		this->IFSETYAW=false;
	}

	this->quaternion=angle2quat(Att(2),Att(1),Att(0));
}
}


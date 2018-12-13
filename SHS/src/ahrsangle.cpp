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

#include "ahrsangle.h"
namespace SHS {
AhrsAngle::AhrsAngle()
{
	 g = 9.8015;
	 integral_signal=-1;//默认安卓用减法
	conv<<0,1,0,1,0,0,0,0,-1;
	INIT_COMPLETE=false;		      
	this->Att<<0,0,0;
	this->IFSETYAW=false;
	this->yaw=0;
	myindex=0;
// 	cout<<"u chose the base and myindex is "<<myindex<<endl;
}




void AhrsAngle::filterInit(double acc_x,double acc_y,double acc_z,double mag_x ,double mag_y,double mag_z)
{



	
}
void AhrsAngle::filterInitByMag(double acc_x, double acc_y, double acc_z, double mag_x, double mag_y, double mag_z)
{
  	if ((acc_x==0 &&acc_y==0&&acc_z==0)||(mag_x==0 &&mag_y==0&&mag_z==0))
	{
		return;
	}
	acc<<acc_x,acc_y,acc_z;
	acc = acc*conv;
	mag<<mag_x,mag_y,mag_z;
	mag = mag*conv;
    // 加速度计确定水平角
    //double Ax = acc(1); Ay = acc(2); Az = acc(3);
    //double Ex = mag(1); Ey = mag(2); Ez = mag(3);
	double pitch = asin(acc(0,0)/this->g);  //  俯仰
    double roll = -asin(acc(0,1)/this->g*cos(pitch));  //  横滚
            // 磁力计确定偏航角
    double Mx = mag(0,0)*cos(pitch) + mag(0,1)*sin(roll)*sin(pitch) + mag(0,2)*cos(roll)*sin(pitch);
    double My = mag(0,1)*cos(roll) - mag(0,2)*sin(roll);
    double yaw = -atan2(My,Mx); 
//             yaw = 0;
            // Set the attitude vector


	this->Att<< roll, pitch, yaw ; // XYZ顺序，即roll pitch yaw顺序

            // Inital attitude
   //         DCM_n2b = eulr2dcm(Att);
   //         DCM_b2n = DCM_n2b';
   //         q = dcm2qua(DCM_b2n);     
			//obj.quaternion = q;
	//初始化四元数
	if (this->IFSETYAW)
	{
		this->Att(2)=this->yaw;
		this->IFSETYAW=false;
	}

	this->quaternion=angle2quat(Att(2),Att(1),Att(0));
	this->INIT_COMPLETE=true;
}


void AhrsAngle::filterUpdate(double gyro_x,double gyro_y,double gyro_z,double acc_x,double acc_y,double acc_z,double mag_x,double mag_y,double mag_z,double deta_time)
{

// 	cout<<"u chose the base and myindex is "<<myindex<<endl;
}
void AhrsAngle::SetYaw(double yaw)
{
	this->IFSETYAW=true;
	this->yaw=yaw;
}
Eigen::Vector3d realy_(0,1,0);
Eigen::Vector3d cury_(0,0,0);
// realy_=realy_*conv;
//   realMinusZ_*=conv;
Eigen::Vector3d realMinusZ_(0,0,1);
Eigen::Vector3d curMinusZ_(0,0,0);
//如果y>0,就是pi/2-theta,否则就是pi/2+theta
//实际想求的是转化完之后与yz平面的夹角
// double AhrsAngle::GetRealYaw()
// {
//   cury_=this->quaternion.inverse()*realy_;
//   double atheta=acos(sqrt(cury_(1)*cury_(1)+cury_(2)*cury_(2)));
//   if(cury_(0)>0)
//   {
//     atheta=atheta;
//   }
//   else
//   {
//     atheta=-atheta;
//   }
//   if(atheta>M_PI)
//   {
//     atheta-=2*M_PI;
//   }
//   else
//   {
//     if(atheta<-M_PI)
//     {
//       atheta+=2*M_PI;
//     }
//   }
//   return atheta;
// 
// }
double AhrsAngle::GetRealYaw()
{
  
  cury_=this->quaternion.inverse()*realy_;
  double atheta=asin(cury_(0));
  if(atheta<0&&cury_(1)<0)
  {
    atheta=-M_PI-atheta;
  }
  else
  {
    if(atheta>=0&&cury_(1)<0)
    {
    atheta=M_PI-atheta;
    }
  }
  
  
  
  if(atheta>M_PI)
  {
    atheta-=2*M_PI;
  }
  else
  {
    if(atheta<-M_PI)
    {
      atheta+=2*M_PI;
    }
  }
  return atheta;

}
double AhrsAngle::GetRealFace()
{

  cury_=this->quaternion.inverse()*realy_;
  double atheta=asin(cury_(0));
  if(atheta<0&&cury_(1)<0)
  {
    atheta=-M_PI-atheta;
  }
  else
  {
    if(atheta>=0&&cury_(1)<0)
    {
    atheta=M_PI-atheta;
    }
  }
  
  atheta+=M_PI/2;
  
  if(atheta>M_PI)
  {
    atheta-=2*M_PI;
  }
  else
  {
    if(atheta<-M_PI)
    {
      atheta+=2*M_PI;
    }
  }
  return atheta;
  
}
AhrsAngle::~AhrsAngle()
{

}
}
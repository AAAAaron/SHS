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

#include "pdrsim.h"

namespace SHS {
  
static vector<PDRPoint> pdresult;
// static	SHS::Step stest;
// static	SHS::AhrsAngle atest;
Eigen::Vector3d wbody(0,0,0);
Eigen::Vector3d wnav(0,0,0);

double inte_wz;
double thisyaw=0;
PDRSIM::PDRSIM()
{
// 	Step::Ptr Stest(new Step);
	
	this->stest = new Step();
	atest=NULL;
	floortest=NULL;
	floorCallBack=NULL;
	this->callback=NULL;
	this->cur_time=0;
	this->cur_index=0;
	pdresult.clear();

	init_x_offset=0;
	init_y_offset=0;
	init_yaw_offset=0;
	mag_offset=0;
	inte_wz=0;
	integral_signal=-1;//默认安卓用减法,苹果=1
	YAWORINTE=YAW;
	pdrEkf=false;//是否使用ekf,ekf要求必须获得有效的姿态，默认使用,12,5先把所有的都不使用新的
	pdrEkfInit();
}

void PDRSIM::InitFloorModule(int curFloorIndex)
{
  floortest = new SHS::AttitudeObserver(curFloorIndex);
}


void PDRSIM::choose_ahrs(int ahrs_index)
{
  switch(ahrs_index){
    case 0:{
//       AhrsAngle::Ptr Artest(new AhrsAngle);//默认方法，不再使用    
//       this->atest = Artest;
      
	this->atest=new AhrsAngle();
//       cout<<this->atest->myindex<<endl;
//       cout<<Artest->myindex<<endl;
      break;}
    case 1:{
//       Gyro::Ptr Artest(new Gyro);//陀螺仪
	this->atest = new Gyro(); 
// 	this->mag_offset=8.0/180.0*M_PI;//偏置度数默认北京8°
      
//       cout<<this->atest->myindex<<endl;
//       cout<<Artest->myindex<<endl;
      break;}
    case 2:{
//       AhrsRobust::Ptr Artest(new AhrsRobust);//带地磁的方法
      this->atest = new AhrsRobust();
//       cout<<this->atest->myindex<<endl;
//       cout<<Artest->myindex<<endl;
      break;}
    case 3:{
      this->atest= new AhrsQsmfFilter();
      break;}
    case 4:{
      this->atest= new AhrsQsmfBFilter();
      break;}  
    case 5:{
      this->atest= new PhoneRotateAngle();
      pdrEkf=false;
      break;}
    default:
      cout<<"no this type"<<endl;
  } 
}

void PDRSIM::pdrEkfInit()
{
  conv<<0,1,0,1,0,0,0,0,-1;
  P = Eigen::Matrix<double,6,6>::Zero();
  Q = Eigen::Matrix<double,6,6>::Zero();
  Fc = Eigen::Matrix<double,6,6>::Zero();
  F = Eigen::Matrix<double,6,6>::Zero();
  for (int i = 0; i < 3; i++)
  {
// 	P(i,i)=pow(1,2);
// 	Q(i,i)=pow(1,2);
    	P(i,i)=1;
	Q(i,i)=1;
  }
  for (int i = 3; i < 6; i++)
  {
// 	P(i,i)=pow(1,2);
// 	Q(i,i)=pow(1,2);
    	P(i,i)=1;
	Q(i,i)=1;
  }  
  Pos_ins_=Eigen::VectorXd::Zero(6);
  step_pvlast_=Eigen::VectorXd::Zero(6);
  Fc(0,3)=1;
  Fc(1,4)=1;
  Fc(2,5)=1;
  Z=Eigen::Vector3d::Zero();
  H=Eigen::Matrix<double,3,6>::Zero();
  H(0,0)=1;
  H(1,1)=1;
  H(2,2)=1;
  R=Eigen::Matrix<double,3,3>::Zero();
  R(0,0)=1*0.01;
  R(1,1)=1*0.01;
  R(2,2)=1*0.01;
//   cout<<"P--"<<P<<endl;
//   cout<<"Q--"<<Q<<endl;
//   cout<<"Fc--"<<Fc<<endl;
//   cout<<"F--"<<F<<endl;
//   cout<<"R--"<<R<<endl;
}
Eigen::VectorXd PDRSIM::Nav_pdr(Eigen::VectorXd& x, Eigen::Vector3d& u, Eigen::Quaterniond& q, double Ts)
{
  Eigen::VectorXd y;
  y=Eigen::VectorXd::Zero(6);
  y<<x(0),x(1),x(2),x(3),x(4),x(5);
  Eigen::Vector3d acc_t;
  acc_t<<0,0,-Gravity;
  u=conv*u;
  acc_t=q*u-acc_t;
  y(3)+=acc_t(0)*Ts;
  y(4)+=acc_t(1)*Ts;
  y(5)+=acc_t(2)*Ts;
  
  //位置更新
  y(0)+=(y(3)+x(3))*0.5*Ts;
  y(1)+=(y(4)+x(4))*0.5*Ts;
  y(2)+=(y(5)+x(5))*0.5*Ts;
  return y;
}


PDRSIM::~PDRSIM()
{
    if(stest!=NULL)
        delete stest;
    if(atest!=NULL)
        delete atest ;
    if(floortest!=NULL)
	delete floortest;
	
}

void PDRSIM::cal_pdr(double sl,double yaw)
{

	PDRPoint cp;
	cp.sl=sl;
	cp.yaw=yaw;	
	if(pdresult.size()==0){
	  cp.x=init_x_offset;
	  cp.y=init_y_offset;
	}
	else{	
	cp.x=pdresult.back().x+sl*sin(yaw);
	cp.y=pdresult.back().y+sl*cos(yaw);
	}
	if(this->cur_index<2)//控制输出的deta角度
	{
	  cp.deta_angle=0;
	}
	else{
	cp.deta_angle=yaw-pdresult.back().yaw;
	}
	pdresult.push_back(cp);
	
}

void PDRSIM::ekf_update(Eigen::Vector3d& u,Eigen::Quaterniond q,double Ts)
{
  Pos_ins_=Nav_pdr(Pos_ins_,u,q,Ts);
  F=Eigen::MatrixXd::Identity(6,6)+Ts*Fc;
  P=F*P*F.transpose()+Q;
//   cout<<Pos_ins_<<endl;
  
}
void PDRSIM::KF_ZHR01(double strideLength, double yaw)
{

  
  
  Z(0)=Pos_ins_(0)-step_pvlast_(0)-strideLength*cos(yaw);
  Z(1)=Pos_ins_(1)-step_pvlast_(1)-strideLength*sin(yaw);
  Z(2)=Pos_ins_(2)-step_pvlast_(2);
//   cout<<step_pvlast_<<endl;
//   cout<<"-------"<<endl;
  
  ////Calculate the Kalman filter gain
  Eigen::MatrixXd Htranspose=Eigen::MatrixXd::Zero(6,3);
  Htranspose= H.transpose();
  K=(P*Htranspose)*((H*P*Htranspose+R).inverse());
  // Estimation of the perturbations in the estimated navigation states      
  dx= K * Z; 
  Pos_ins_-=dx;
  P=(Eigen::MatrixXd::Identity(6,6)-K*H)*P;
  P=(P+P.transpose())/2.0;
  
    	PDRPoint cp;
	cp.x=Pos_ins_(1)+init_x_offset;
	cp.y=Pos_ins_(0)+init_y_offset;
	cp.sl=strideLength;
	cp.yaw=yaw;

	if(this->cur_index<2)//控制输出的deta角度
	{
	  cp.deta_angle=0;
	}
	else{
	cp.deta_angle=yaw-pdresult.back().yaw;
	}
	pdresult.push_back(cp);
  
	for(int index=0;index<6;index++){
	step_pvlast_(index)=Pos_ins_(index);//数值复制	
	}	
  

}


void PDRSIM::InitialXYYaw(double x, double y, double yaw)
{
	this->cur_index=0;
	pdresult.clear();
	init_x_offset=x;
	init_y_offset=y;
	init_yaw_offset=yaw;
	atest->INIT_COMPLETE=false;
	atest->SetYaw(yaw);//可能起不了作用
	thisyaw=yaw;

}
void PDRSIM::InitialXYYawFloor(double x, double y, double yaw, int curFloorIndex)
{
  InitialXYYaw(x,y,yaw);
  InitFloorModule(curFloorIndex);
}


void PDRSIM::InitialXY(double x,double y)
{
	this->cur_index=0;
	pdresult.clear();
	init_x_offset=x;
	init_y_offset=y;
	
}

bool PDRSIM::adddata(double gyro_x,double gyro_y,double gyro_z,double linear_acc_x,double linear_acc_y,double linear_acc_z,double gx,double gy,double gz,double mag_x,double mag_y,double mag_z,double time_r)
{
  
	//acc_x是全部的加速度
      if(integral_signal==1){
// 	gyro_x*=-1;
// 	gyro_y*=-1;
// 	gyro_z*=-1;
	gx*=-Gravity;
	gy*=-Gravity;
	gz*=-Gravity;
	linear_acc_x*=-Gravity;
	linear_acc_y*=-Gravity;
	linear_acc_z*=-Gravity;	
	
	time_r/=1e+6;
	
      }

	stest->add_data(linear_acc_x+gx,linear_acc_y+gy,linear_acc_z+gz,time_r);

		//2018-10-24理论上讲是不是应该传入重力的三向加速度矢量更准确，现在传入的是所有的加速度，那么是不是更容易带来误差，但是结果看是差别不是很大
		//如果刚开始滤波不完整的话，重力分量可能是有偏差的，看起来能及时应该收敛
	atest->filterUpdate(gyro_x,gyro_y,gyro_z,gx,gy,gz,mag_x,mag_y,mag_z,time_r-this->cur_time);
	
	switch(YAWORINTE)
	{
	  case INTEGRAL:{
// 	    cout<<"INTEGRAL"<<endl;
	    if(cur_time>0){
	    wbody<<gyro_x,gyro_y,gyro_z;
	    wnav = atest->quaternion*wbody;
// 	    thisyaw+=integral_signal*wnav(2)*(time_r-cur_time);
// 	    inte_wz+=integral_signal*wnav(2)*(time_r-cur_time);
	    thisyaw+=-1*wnav(2)*(time_r-cur_time);
	    inte_wz+=-1*wnav(2)*(time_r-cur_time);
// 	    if(pdresult.size()==0){
// 	      thisyaw+=integral_signal*inte_wz;
// 	    }
// 	    else{
// 	      thisyaw=pdresult.back().yaw+integral_signal*inte_wz;//考虑到安卓的应该用减法
// 	    }
	    }
	    break;
	  }
	  case YAW:{
// 	    cout<<"YAW"<<endl;
	    thisyaw=atest->Att(2);
	    thisyaw+=mag_offset;
	    break;
	  }
	  case INTERANGLE:{
// 	    cout<<"INTERANGLE"<<endl;
	    thisyaw=atest->GetRealYaw();
	    thisyaw+=mag_offset;
	    break;
	  }
	  default:
	    cout<<"no this out angle type!"<<endl;
	    break;
	}
	
	
	Eigen::Vector3d g(gx+linear_acc_x,gy+linear_acc_y,gz+linear_acc_z);
// 	cout<<"att"<<this->atest->Att<<endl;
	if(cur_time!=0&&pdrEkf){
	ekf_update(g,atest->quaternion,time_r-this->cur_time);
	}
	
	if (stest->ISSTEP)
	{
		this->cur_index++;
		
		if(!pdrEkf){
		this->cal_pdr(stest->stride_length,thisyaw);//测试步长的ekf
		}
		
// 		cout<<inte_wz<<endl;
// 		cout<<"wnav"<<wnav<<endl;

		long int ts= pdresult.size();
		bool xs=callback!=NULL;
		if(xs&&(ts>0))
		{
// 		  cout<<pdresult.back().deta_angle<<endl;
 		(*callback)(pdresult.back().x,pdresult.back().y,pdresult.back().sl,pdresult.back().yaw,pdresult.back().deta_angle);
		}
		inte_wz=0;
		if(cur_index>0&&pdrEkf){
		  
		  KF_ZHR01(stest->stride_length,thisyaw);

		}
		else{
		  	for(int index=0;index<6;index++){
			  step_pvlast_(index)=Pos_ins_(index);//数值复制	
			}
		}
	}
	this->ISSTEP=stest->ISSTEP;
	
	this->cur_time=time_r;
	
	return this->ISSTEP;
}

double PDRSIM::get_X()
{
	return pdresult.back().x;
}
double PDRSIM::get_Y()
{
	return pdresult.back().y;
}
double PDRSIM::get_SL()
{
	return pdresult.back().sl;
}
double PDRSIM::get_YAW()
{
	return pdresult.back().yaw;
}
double PDRSIM::get_deta_angle()
{
	return pdresult.back().deta_angle;
}
//必要时重启角度滤波器
void PDRSIM::reset_angel()
{
	atest->INIT_COMPLETE=false;
}
void PDRSIM::setCallBack(PDRSIMCALLback fun)
{
	this->callback=fun;
}
void PDRSIM::setFloorCallBack(PDRSIM::FLOORCALLBACK fun)
{
	floorCallBack=fun;
}


void PDRSIM::set_magoffset(double offset)
{
	this->mag_offset=offset;
}

void PDRSIM::setXY(double x, double y)
{
	if(pdresult.size()==0){
	  init_x_offset=x;
	  init_y_offset=y;
	}
	else{
	pdresult.back().x=x;
	pdresult.back().y=y;
	}
}

void PDRSIM::setYaw(double yaw)
{
  if(pdresult.size()==0){
     atest->INIT_COMPLETE=false;
     atest->SetYaw(yaw);
     init_yaw_offset=yaw;
  }
  else{
     pdresult.back().yaw=yaw;
  }
  thisyaw=yaw;
}

void PDRSIM::InitYaw(double yaw)
{
     atest->INIT_COMPLETE=false;
     atest->SetYaw(yaw);
     init_yaw_offset=yaw;
} 

void PDRSIM::set_OutAngle(int OutAngleIndex)
{
    this->YAWORINTE=OutYaw(OutAngleIndex);
}
void PDRSIM::set_IOS()
{
  integral_signal=1;
  this->atest->integral_signal=1;
}

void PDRSIM::floorModuleAddFloorHeight(double floor_height)
{
  if(floortest!=NULL)
  {
    floortest->addFloorHeight(floor_height);
  }
  else
  {
    cout<<"还未初始化楼层模块"<<endl;
  }
}
void PDRSIM::floorModuleUpdateFloorHeightMatrix()
{
  if(floortest!=NULL)
  {
    floortest->updateFloorHeightMatrix();
  }
  else
  {
    cout<<"还未初始化楼层模块"<<endl;
  }
}
int PDRSIM::floorModuleAddData(double accNorm, double orientation, double pressure, double time_c)
{
  int res=floortest->addData(accNorm,orientation,pressure,time_c);
  if(floortest->ISCHANGEFLOOR!=0)
  {
  (*floorCallBack)(floortest->FLOORTYPE,floortest->ISCHANGEFLOOR,floortest->k_temp,floortest->accChange_,floortest->currentFloorIndex);
  }
  return res;
}

  
}
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

#include "ahrsrobust.h"
namespace SHS {
AhrsRobust::AhrsRobust()
{
		//quaternion;		
        QSF_Mag=false;       
	P = Eigen::Matrix<double,4,4>::Zero();
        Q = Eigen::Matrix<double,4,4>::Zero();       
   
	noises_att = 0.1*Eigen::Vector4d::Ones();     
	noises_gyroscope = 0.0001*Eigen::Vector4d::Ones(); 

        noises_accelerometer = 0.03*Eigen::Vector3d::Ones()*3;	
        noises_magnetometer = 0.3*Eigen::Vector3d::Ones()*5;   
        
        QSF_Mag_Window = 100;
		//QSF_Mag_Values = [];
		//QSF_Mag_Norm_Values = [];
        QSF_Mag_mean = 3;
		QSF_Mag_gamma = 3;
		QSF_Mag_outlier = 8; 
		for (int i = 0; i < 4; i++)
		{
			this->P(i,i)=pow(noises_att(i),2)*100;
			this->Q(i,i)=pow(noises_gyroscope(i),2);
		}
		//this->P = diag(obj.noises_att).^2*100;           
		//this->Q = diag(obj.noises_gyroscope).^2;
		myindex=2;
		cout<<"u chose the ahrsrobust and cons "<<myindex<<endl;
}

void AhrsRobust::filterInit(double acc_x, double acc_y, double acc_z, double mag_x, double mag_y, double mag_z)
{
  
  SHS::AhrsAngle::filterInitByMag(acc_x,acc_y,acc_z,mag_x,mag_y,mag_z);
}
void AhrsRobust::filterUpdate(double gyro_x, double gyro_y, double gyro_z, double acc_x, double acc_y, double acc_z, double mag_x, double mag_y, double mag_z, double deta_time)
{	
//   cout<<"u chose the ahrsrobust filter and myindex is "<<myindex<<endl;
//   cout<<this->Att<<endl;
	this->QSF_Mag=false;
	dataPoint datmp;
	double mean1 = 0., variance1 = 0., stddev1 = 0.; 
	
	if ((acc_x==0 &&acc_y==0&&acc_z==0)||(mag_x==0 &&mag_y==0&&mag_z==0)||(gyro_x==0 &&gyro_y==0&&gyro_z==0))
	{
		return;
	}
	if (!this->INIT_COMPLETE)
	{
		this->filterInit(acc_x,acc_y,acc_z,mag_x ,mag_y,mag_z);
		return;
	}
	//坐标系转化
	gyro<<gyro_x,gyro_y,gyro_z;
	gyro=gyro*conv;
	acc<<acc_x,acc_y,acc_z;
	acc = acc*conv;
	mag<<mag_x,mag_y,mag_z;
	mag = mag*conv;

 	double mag_magn=sqrt(mag_x*mag_x+mag_y*mag_y+mag_z*mag_z);
	
	//单纯依赖值不是很可靠，所以设置了很小的范围，正常情况下不会进来
//   	if(mag_magn>200 || mag_magn<10)
//   	{
//   	  this->QSF_Mag=false;
//   	}
	
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
	this->q_apriori =this->quaternion*q_r;
	this->q_apriori.normalize();
	////Time updata-----------

	this->x_apriori<<q_apriori.w(),q_apriori.x(),q_apriori.y(),q_apriori.z();

	F=Eigen::Matrix<double,4,4>::Zero();
	F<<0 , -gyro(0),-gyro(1),-gyro(2) ,
		gyro(0) , 0 , gyro(2), -gyro(1) ,
		gyro(1) ,-gyro(2),0 ,gyro(0) ,
		gyro(2),gyro(1),-gyro(0),    0;

	Phi=Eigen::Matrix<double,4,4>::Identity()+deta_time/2.0*F;
	P_apriori= Phi*this->P*Phi.transpose()+this->Q*deta_time;
	// 磁干扰判断 % 模值 + 陀螺和磁力差角(不稳定，已经删除)
// 	double normMag=sqrt(mag_x*mag_x+mag_y*mag_y+mag_z*mag_z);
// 	if(QSF_Mag_Norm_Values.size()>=QSF_Mag_Window)
// 	{
// 	  QSF_Mag_Norm_Values.pop_front();
// 	}
// 	QSF_Mag_Norm_Values.push_back(normMag);
// 	meanStdDev(QSF_Mag_Norm_Values, &mean1, &variance1, &stddev1);
// 	auto maxPosition = max_element(QSF_Mag_Norm_Values.begin(), QSF_Mag_Norm_Values.end());
// 	auto minPosition = min_element(QSF_Mag_Norm_Values.begin(), QSF_Mag_Norm_Values.end());
// 	double difvalue=*maxPosition-*minPosition;
// 	if(variance1<QSF_Mag_gamma&&abs(normMag-mean1)<=QSF_Mag_mean&&difvalue<QSF_Mag_outlier)
// 	{
// 	  QSF_Mag=true;
// 	}
// 	else{
// 	  QSF_Mag=false;
// 	}
	
	h=q_apriori*mag;
	b_n =Eigen::Vector3d::Zero();
	b_n(0)=sqrt(pow(h(0),2)+pow(h(1),2));
	b_n(2)=h(2);
	g_n=Eigen::Vector3d::Zero();
	g_n(2)=-this->g;
	if (this->QSF_Mag)//禁止使用磁力计参数预留位置
	{
		Z=Eigen::VectorXd::Zero(6);
		Eigen::Map<Eigen::Vector3d> acc_vector(acc.data(),acc.size());
		Eigen::Map<Eigen::Vector3d> mag_vector(mag.data(),mag.size());
		Z<<acc_vector- q_apriori.conjugate()*g_n,mag_vector-q_apriori.conjugate()*b_n;
		
		H = Eigen::MatrixXd::Zero(6,4);
		H<<jacobianES(q_apriori, g_n, "long"),jacobianES(q_apriori, b_n,"long");

		Htranspose=Eigen::MatrixXd::Zero(4,6);
		Htranspose= H.transpose();
		R=Eigen::MatrixXd::Zero(6,6);
		R(0,0)=pow(this->noises_accelerometer(0),2);
		R(1,1)=pow(this->noises_accelerometer(1),2);
		R(2,2)=pow(this->noises_accelerometer(2),2);

		R(3,3)=pow(this->noises_magnetometer(0),2);
		R(4,4)=pow(this->noises_magnetometer(1),2);
		R(5,5)=pow(this->noises_magnetometer(2),2);


		//R<<this->noises_accelerometer.diagonal().cwiseProduct(this->noises_accelerometer.diagonal());
		////Calculate the Kalman filter gain
		K=(P_apriori*Htranspose)*((H*P_apriori*Htranspose+R).inverse());  
		
		// Estimation of the perturbations in the estimated navigation states      
		detax= K * Z;   

		X_state =x_apriori+ detax; 
		// Update the filter state covariance matrix P.
		this->P = (Eigen::MatrixXd::Identity(4,4) - K*H) * P_apriori;
        // Make sure the filter state covariance matrix is symmetric. 
		P = (P+P.transpose())/2.0;

		X_state.normalize();
		this->quaternion = Eigen::Quaterniond(X_state(0),X_state(1),X_state(2),X_state(3));
		this->Att =quat2angle(this->quaternion);

	}
	else
	{
		Z=Eigen::VectorXd::Zero(3);
		Eigen::Map<Eigen::Vector3d> acc_vector(acc.data(),acc.size());
		Z<<acc_vector- q_apriori.conjugate()*g_n;
		H = Eigen::MatrixXd::Zero(3,4);
		H<<jacobianES(q_apriori, g_n, "long");
		Htranspose=Eigen::MatrixXd::Zero(4,3);
		Htranspose= H.transpose();
		R=Eigen::MatrixXd::Zero(3,3);
		R(0,0)=pow(this->noises_accelerometer(0),2);
		R(1,1)=pow(this->noises_accelerometer(1),2);
		R(2,2)=pow(this->noises_accelerometer(2),2);

		///Calculate the Kalman filter gain
		K=(P_apriori*Htranspose)*((H*P_apriori*Htranspose+R).inverse());           
		// Estimation of the perturbations in the estimated navigation states      
		detax= K * Z;              
		X_state =x_apriori+ detax;       
		// Update the filter state covariance matrix P.
		this->P = (Eigen::MatrixXd::Identity(4,4) - K*H) * P_apriori;
        // Make sure the filter state covariance matrix is symmetric. 
		P = (P+P.transpose())/2;
		X_state.normalize();

		this->quaternion = Eigen::Quaterniond(X_state(0),X_state(1),X_state(2),X_state(3));
		this->Att =quat2angle(this->quaternion);

	//	Eigen::Vector3d Att2 =quat2angle( Eigen::Quaterniond(x_apriori(0),x_apriori(0),x_apriori(0),x_apriori(0)));
	//	this->Att(1)=Att2(1);
	//	this->Att(0)=Att2(0);
	//	this->quaternion=angle2quat(Att(2),Att(1),Att(0));
	}
	
	if (this->IFSETYAW)
	{
		this->Att(2)=this->yaw;
		this->IFSETYAW=false;
	}
	this->quaternion=angle2quat(Att(2),Att(1),Att(0));
}

AhrsRobust::~AhrsRobust()
{

}  
}


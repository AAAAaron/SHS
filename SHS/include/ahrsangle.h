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
/**
 * 用于作为角度滤波器的初始类,默认实现的是纯陀螺的积分方法
 */
#ifndef AHRSANGLE_H
#define AHRSANGLE_H
#include "common_include.h"

namespace SHS
{
class AhrsAngle
{
public:
// 	typedef shared_ptr<AhrsAngle> Ptr;
	int integral_signal;//标示要加减的问题，应该是安卓减，苹果加
	bool INIT_YAW;
	double IFSETYAW;
	bool INIT_COMPLETE;
	double yaw;
	int myindex;
	Eigen::Matrix<double,3,3> conv ;
	Eigen::Quaternion<double > quaternion;		
        double g;        
	Eigen::MatrixXd P ;
	Eigen::MatrixXd Q;       
        // initial state covariance matrix (P).  
	Eigen::Vector4d noises_att; 
	Eigen::Vector3d noises_bias;
        // Q
        Eigen::Vector4d noises_gyroscope;
	Eigen::Vector3d  noises_gyroscopeBias   ;  
        //meas nosise R
        Eigen::Vector3d noises_accelerometer;	
        Eigen::Vector3d noises_magnetometer ;   
	
	Eigen::Matrix<double,1,3> gyro;	
	Eigen::Matrix<double ,1,3> acc;
	Eigen::Matrix<double ,1,3>mag;
	Eigen::Matrix<double,1,3> Angle_bn_b;
	Eigen::Vector3d Att ;
	
	Eigen::Vector3d MagRef;
	deque<dataPoint> QSF_Mag_Values;
	deque<double> QSF_Mag_Norm_Values;
public:	
	
	AhrsAngle();
	~AhrsAngle();
	virtual void filterInit(double acc_x,double acc_y,double acc_z,double mag_x ,double mag_y,double mag_z);
	void filterInitByMag(double acc_x,double acc_y,double acc_z,double mag_x ,double mag_y,double mag_z);
	virtual void filterUpdate(double gyro_x,double gyro_y,double gyro_z,double acc_x,double acc_y,double acc_z,double mag_x,double mag_y,double mag_z,double deta_time);
	void SetYaw(double yaw);
	double GetRealYaw();//生成与真y的夹角，或者说与启动y的夹角
	double GetRealFace();//在相机旋转的过程中，生成面向的角度
protected:
  //for kalman filter
	Eigen::Quaterniond q_r;
	Eigen::Quaterniond q_apriori;
	Eigen::Vector4d x_apriori;
	Eigen::Vector4d X_state;
	Eigen::MatrixXd F ;
	Eigen::MatrixXd Phi;
	Eigen::MatrixXd P_apriori ;
	Eigen::Vector3d h;
	Eigen::Vector3d b_n;
	Eigen::Vector3d g_n;
	Eigen::VectorXd Z;
	Eigen::MatrixXd H;
	Eigen::MatrixXd Htranspose;
	Eigen::MatrixXd R;
	Eigen::MatrixXd K;
	Eigen::MatrixXd detax;
  //for magnetic 
	bool QSF_Mag;
	int QSF_Mag_Window ;

	
        double QSF_Mag_mean ;
	double QSF_Mag_gamma ;
	double QSF_Mag_outlier ; 
	
	Eigen::Vector3d bias;
private:
};
}
#endif // AHRSANGLE_H

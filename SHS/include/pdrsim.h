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

#ifndef PDRSIM_H
#define PDRSIM_H
#include "common_include.h"
#include "ahrsangle.h"
#include "ahrsrobust.h"
#include "ahrsqsmffilter.h"
#include "ahrsqsmfbfilter.h"
#include "phonerotateangle.h"
#include "step.h"
#include "gyro.h"
#include "attitudeobserver.h"
namespace SHS
{

class PDRSIM
{
public: 
	typedef void (*PDRSIMCALLback)(double,double,double,double,double);
	typedef void (*FLOORCALLBACK)(int,int,double,double,int);//时间,换层类型（1和-1上下）,k,std_accnorm.当前到达层索引
// 	Step::Ptr stest;
// 	AhrsAngle::Ptr atest;
	Step* stest;
	AhrsAngle* atest;
	AttitudeObserver* floortest;
	PDRSIMCALLback callback;
	FLOORCALLBACK floorCallBack;
	double cur_time;
	int cur_index;
	bool pdrEkf;
	bool ISSTEP=false;
	OutYaw YAWORINTE;//积分1还是直接使用yaw，默认是yaw0,求解的和yz平面的夹角是2，但是注意的是
	double mag_offset;
	double init_yaw_offset;
	double init_x_offset;
	double init_y_offset;
	int integral_signal;//标示要加减的问题，应该是安卓减，苹果加
	void setCallBack(PDRSIMCALLback fun);	
	void setFloorCallBack(FLOORCALLBACK fun);
	void InitialXY(double x,double y);
	void InitialXYYaw(double x,double y,double yaw);//初始值设置，可以不调用，也可以只初始化xy，其中yaw可以默认，yaw角空置即使用默认的角度
	void InitialXYYawFloor(double x,double y,double yaw,int curFloorIndex);//三个模块全部初始化
	void InitYaw(double yaw);
	void InitFloorModule(int curFloorIndex);
	void setXY(double x,double y);
	void setYaw(double yaw);
	bool adddata(double gyro_x,double gyro_y,double gyro_z,double linear_acc_x,double linear_acc_y,double linear_acc_z,double gx,double gy,double gz,double mag_x,double mag_y,double mag_z,double time_r);//添加分离的加速度数据
	void floorModuleAddFloorHeight(double floor_height);
	void floorModuleUpdateFloorHeightMatrix();
	int floorModuleAddData(double accNorm,double orientation,double pressure,double time_c);
	void reset_angel();//重新启动角度滤波器
	void pdrEkfInit();
	double get_X();
	double get_Y();
	double get_SL();
	double get_YAW();
	double get_deta_angle();
	double get_mx();
	double get_my();
	double get_mz();
	double get_time();
	void set_magoffset(double offset);
	void set_IOS();
	void choose_ahrs(int ahrs_index);
	void set_OutAngle(int OutAngleIndex);
	PDRSIM();
	~PDRSIM();

private:
// 	Step stest;
// 	AHRSROBUST atest;
	void cal_pdr(double sl,double yaw);
	void ekf_update(Eigen::Vector3d& u,Eigen::Quaterniond q,double Ts);
	Eigen::VectorXd Nav_pdr(Eigen::VectorXd &x,Eigen::Vector3d &u,Eigen::Quaterniond &att,double Ts);
	void KF_ZHR01(double strideLength,double yaw);
	Eigen::Matrix3d conv;
	Eigen::Matrix <double,6,6> P;
	Eigen::Matrix <double,6,6> Q;
	Eigen::Matrix <double,6,6> Fc;
	Eigen::Matrix <double,6,6> F;
	Eigen::Vector3d Z;
	Eigen::Matrix <double,3,6> H;
	Eigen::Matrix <double,3,3> R;
	Eigen::MatrixXd K;
	Eigen::MatrixXd dx;
	Eigen::VectorXd Pos_ins_;
	Eigen::VectorXd step_pvlast_;
	Eigen::Vector3d magAfterRotate;
	
  
};
}
#endif // PDRSIM_H

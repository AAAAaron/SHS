/*用于计算步长
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
 */

#ifndef STEP_H
#define STEP_H
#include "common_include.h"
namespace SHS
{
class Step
{
public:
// 	typedef shared_ptr<Step> Ptr;
	double K11;
	double b11;
	double step_time_threshold_min;
	double step_time_threshold_max;
	double FS;//采样频率FrequeOfSample
	double Wn;//截止频率cutoff
	unsigned int but_length_Fs;//缓存数据长度，用于计算滤波频率
	double b[5];
	double a[4];
	int filter_order;//巴特沃斯低通滤波器阶数
	double umbral_Acc; // 峰值最小阈值
	double umbral_Acc_descarte; //% 峰值最大阈值
	double gravity;//重力
	bool ISSTEP;//标识是否为步
	double stride_length;//步长
	unsigned int cout_vector;
	
	vector<dataPoint > acc;
	vector<int> step_index;
	//int start_index;
	vector<double > acc_mag_filtered;
	
	Step(void);
	~Step(void);
	bool add_data(double acc_x,double acc_y,double acc_z ,double time_add);
	double Step_length(const vector<dataPoint> &caldata);
	void filter_para();
	void set_para(double k11_scale,double b11_scale);
};
}
#endif // STEP_H

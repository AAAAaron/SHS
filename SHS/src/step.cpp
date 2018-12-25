/*step的实现
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

#include "step.h"
#include "btwfilter.h"
namespace SHS {
  
Step::Step(void)
{
  	K11 = 0.111153867959715;
	b11 = 0.211140400598447;
	this->step_index.clear();
	this->filter_order=4;
	this->but_length_Fs=100;
	this->FS=0;

	this->umbral_Acc=0.15; // 峰值最小阈值
	this->umbral_Acc_descarte=15.0; // 峰值最大阈值
	this->gravity=9.8; 
	this->step_index.push_back(0);
	//this->start_index=0;//标识起点
	step_time_threshold_max=0.8;
	step_time_threshold_min=0.25;
	this->cout_vector=0;
}


Step::~Step(void)
{
}

//不为0是置入,时间输入秒
bool Step::add_data(double acc_x,double acc_y,double acc_z ,double time_add)
{
	this->ISSTEP=false;
	dataPoint tmp;
	tmp.data_x = acc_x;
	tmp.data_y = acc_y;
	tmp.data_z =acc_z;
	tmp.time_r=time_add;
	double mag=sqrt(acc_x*acc_x+acc_y*acc_y+acc_z*acc_z);
	tmp.magnitude=mag;

	tmp.detection_binary=0;
	gravity = 0.999*gravity+0.001*mag;
	
	//unsigned int cout_vector=acc.size();
	double detrend=0;
	
// 	if (acc.size()>but_length_Fs)
	if (abs(FS-0)>1e-5)	  
	{
		//butterworth_filter
		double tmpmag=mag*b[4];
		for (int i = 0; i < this->filter_order; i++)
		{
			tmpmag+=acc[acc.size()-1-i].magnitude*b[this->filter_order-1-i]-a[i]*acc_mag_filtered[acc.size()-1-i];
		}
		acc_mag_filtered.push_back(tmpmag);
		detrend=tmpmag-gravity;
	}
	else
	{
		acc_mag_filtered.push_back(mag);
		detrend=mag-gravity;//分离重力
	}
	if (abs(FS-0)<1e-9&& acc.size()==but_length_Fs )//计算频率只调用一次
	{	
		filter_para();
	}

	
	if(cout_vector>1&&abs(FS-0)>1e-5)	//标注峰谷情况并计算步长
	{
		if (detrend>this->umbral_Acc && detrend<this->umbral_Acc_descarte)
		{
			tmp.detection_binary=1;
		}
		else
		{
			if (detrend<-umbral_Acc)
			{
				if (acc[acc.size()-2].detection_binary==1)
				{
					tmp.detection_binary=0;//波中
				}
				else
				{
					tmp.detection_binary=-1;
				}
			}
			else
			{
				tmp.detection_binary=0;
			}
		}

		
		//计算是否为1步
		if (tmp.detection_binary==-1&& acc.back().detection_binary==0)
		{
			for (unsigned int i =acc.size()-(cout_vector- this->step_index.back()); i < acc.size(); i++)
			{
				if (acc[i].detection_binary==1)
				{
								
					this->ISSTEP=true;
					vector <dataPoint> tmp(acc.end()-(cout_vector-this->step_index.back()),acc.end());
					this->stride_length=this->Step_length(tmp);
// 					cout<<cout_vector<<"------"<<endl;
					this->step_index.push_back(cout_vector);
					break;
				}
			}
		}
		acc.push_back(tmp);
	}
	else
	{
		acc.push_back(tmp);
	}
	
	if (this->step_index.size()>3)
	{
		int dif_index =this->step_index[1]-this->step_index[0];
		this->acc.erase(this->acc.begin(),this->acc.begin()+dif_index);
		this->acc_mag_filtered.erase(this->acc_mag_filtered.begin(),this->acc_mag_filtered.begin()+dif_index);
		this->step_index.erase(this->step_index.begin());
	}
	this->cout_vector++;
	return this->ISSTEP;
}

double Step::Step_length(const vector<dataPoint> &caldata)
{

	//时间长的先去掉
	double dt=caldata.back().time_r-caldata[0].time_r;
	if (dt>this->step_time_threshold_max||dt<this->step_time_threshold_min)
	{
		return 0.0;
	}

	double min=1000;
	double max=-1000;
	for (unsigned int i = 0; i < caldata.size(); i++)
	{
		if (caldata[i].magnitude<min)
		{
			min=caldata[i].magnitude;
		}
		if (caldata[i].magnitude>max)
		{
			max=caldata[i].magnitude;
		}
	}
	double peak=max-min;
	if (peak<=0)
	{
		return 0.0;
	}
	double sl=pow(peak,0.25)/pow(dt,1)*K11 + b11; 
	return sl;
}

void Step::filter_para()
{
	if (this->acc.size()<this->but_length_Fs)
	{
		
	}
	else
	{
		this->FS=this->but_length_Fs/(this->acc.back().time_r-this->acc[acc.size()-but_length_Fs].time_r);
		//this->FS=263;
		cout<<"filter="<<this->FS<<endl;
		BtwFilter btwtest;
		double wn[2] = { 2.5/(FS/2.0), 0.0};
		double aa[5];
		btwtest.mybutter(this->filter_order, wn, 1, 0, aa, b);

		//b[0]=b;
		//b[1]=GAIN*4;
		//b[2]=GAIN*6;
		//b[3]=GAIN*4;
		//b[4]=GAIN;

		this->a[0]=aa[1];
		this->a[1]=aa[2];
		this->a[2]=aa[3];
		this->a[3]=aa[4];
	}

}
void Step::set_para(double k11_scale, double b11_scale)
{
  K11*=k11_scale;
  b11*=b11_scale;
}


}
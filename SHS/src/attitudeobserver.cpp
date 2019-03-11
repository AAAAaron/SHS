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
 * 2018,12,03
 */

#include "attitudeobserver.h"
namespace SHS{
AttitudeObserver::AttitudeObserver(int floorId)
{
  FS=0;//未设置时是0；
  initTime_=0;
  currentFloorIndex=floorId;//初始化要求输入当前层在层指纹中的序号
  currentDataIndex_=0;
  point_count_=0;
  pro_time=3;
  floorHeight_.clear();
  recordPressure_=0;
  ISCHANGEFLOOR=0;
  FLOORTYPE=0;
  threshold = 0.06;//阈值
  factor = 0.8;//楼层高度的比例
  factorBase = 0.1;//刷新基线的要求
  floorHeading=0;//默认的方向是0
}
AttitudeObserver::AttitudeObserver()
{
  FS=0;//未设置时是0；
  initTime_=0;
  currentFloorIndex=-100;//初始化要求输入当前层在层指纹中的序号,不使用默认是-100
  currentDataIndex_=0;
  point_count_=0;
  pro_time=3;
  floorHeight_.clear();
  recordPressure_=0;
  ISCHANGEFLOOR=0;
  FLOORTYPE=0;
  threshold = 0.06;//阈值
  factor = 0.8;//楼层高度的比例
  factorBase = 0.1;//刷新基线的要求
  floorHeading=0;//默认的方向是0
}

AttitudeObserver::~AttitudeObserver()
{

}
int difBuffLength=0;
int proCalLength=4;
//这个函数会返回是否是上下楼，上下是-1,1，确认顺序反人类,默认是0，出现问题是-100
int AttitudeObserver::addData(double accNorm,double orientation ,double pressure, double time_c)
{
  if(currentFloorIndex==-100)
  {
    return -100;
  }
  currentDataIndex_++;
  ISCHANGEFLOOR=0;
  if(abs(pressure)<=1e-5 ||abs(accNorm)<1e-5){
    return -100;
  }
  
  dataPoint cp;
//   memset();
  cp.data_z=pressure;
  cp.time_r=time_c;
  cp.data_x=accNorm;//data_x作为加速度摸值
  cp.data_y=orientation;//data_y是方向角
  
  if(initTime_==0){
    initTime_=time_c;
    recordTime_=time_c;
    cp.value_flitered=pressure;
    point_count_++;
    calDifBuff.push_back(pressure);
  }
  else{
    //第一个3s内的处理方式
    if(time_c-initTime_<=pro_time){  
      //Mk=(M(k-1)*(k-1)+Pk)/k;
      cp.value_flitered=pressure_.back().value_flitered*pressure_.size()+pressure;
      cp.value_flitered/=(pressure_.size()+1);
      point_count_++;
    }
    //3s后的3处理方式
    else{
      //数据进入时可能处于一个非常不稳定的状态
      if(abs(recordPressure_)<1e-9)
      {
// 	recordPressure_=std::accumulate(calDifBuff.begin(),calDifBuff.end(),0.0);
	
	recordPressure_=0;
	for(unsigned int jf=0;jf<calDifBuff.size();jf++)
	{
	  recordPressure_+=calDifBuff[jf];
	}
	recordPressure_/=calDifBuff.size();

      }

      cp.value_flitered+=pressure_.back().value_flitered;
      cp.value_flitered+=(pressure-pressure_[pressure_.size()-point_count_].data_z)/point_count_;
      
    }
    
    calDifBuff.push_back(pressure); 
    if(time_c-recordTime_>pro_time)
    {
      recordTime_=time_c;
      double max_calDifBuff=*max_element(calDifBuff.begin(),calDifBuff.end());
      double min_calDifBuff=*min_element(calDifBuff.begin(),calDifBuff.end());
      double difMaxMin=max_calDifBuff-min_calDifBuff;
      
      difBuffer.push_back(difMaxMin);
//       cout<<"difMaxMin"<<difMaxMin<<endl;
      if(difBuffer.size()>=proCalLength)
      {
	double max_difBuffer=*max_element(difBuffer.end()-proCalLength,difBuffer.end()-1);
      
	if (max_difBuffer>threshold&&difBuffer.back()<=threshold)//进入稳态条件
	{
// 	  cout<<time_c-initTime_<<".."<<difBuffer.size()<<endl;
	  double deltaHeight = -(pressure-recordPressure_) / 0.125f;
	  if(currentFloorIndex > 0 && deltaHeight < -floorHeight_[currentFloorIndex-1]*factor) 
	  {
	  //下楼,首先根据最小绝对值判断到达的楼层
	    int minFloorIndex=0;
	    double minDist=1e+5;
	    for(int i=0;i<=currentFloorIndex;i++)
	    {
	      double thisDist=abs(deltaHeight- heightMatrix_[currentFloorIndex][i]);
	      if(thisDist <minDist)
	      {
		minDist=thisDist;
		minFloorIndex=i;
	      }
	    }
	    
	    if(currentFloorIndex!=minFloorIndex)//如果索引不一致就换，索引一致还是不换
	    {
	      currentFloorIndex=minFloorIndex;
	      ISCHANGEFLOOR =1;//1是下楼  注意这里是原杜哥算法给出的值，与原来的印象不同
	      calculateChangeFloorType();
	      recordPressure_=pressure;	 

	    }

	  }
	  if(currentFloorIndex<floorHeight_.size()-1 && deltaHeight>floorHeight_[currentFloorIndex-1]*factor)
	  {
	  //上楼,首先根据最小绝对值判断到达的楼层
	    int minFloorIndex=0;
	    double minDist=1e+5;
	    for(int i=currentFloorIndex;i<floorHeight_.size();i++)
	    {
	      double thisDist=abs(deltaHeight- heightMatrix_[currentFloorIndex][i]);
	      if(thisDist <minDist)
	      {
		minDist=thisDist;
		minFloorIndex=i;
	      }
	    }
	    if(currentFloorIndex!=minFloorIndex)//如果索引不一致就换，索引一致还是不换
	    {
	    currentFloorIndex=minFloorIndex;
	    ISCHANGEFLOOR =-1;//-1是上楼  	 
	    calculateChangeFloorType();
	    recordPressure_=pressure;

	    }
	  }
	}
	else//未进入稳态则刷新基线值
	{
	  if(difBuffer[difBuffer.size()-2]<threshold*factorBase)
	  {
// 	    recordPressure_=std::accumulate(calDifBuff.begin(),calDifBuff.end(),0.0);
// 	    recordPressure_/=calDifBuff.size();	 
	    recordPressure_=0;
	    for(unsigned int jf=0;jf<calDifBuff.size();jf++)
	    {
	      recordPressure_+=calDifBuff[jf];
	    }
	    recordPressure_/=calDifBuff.size();
	  }
	}

      }

      calDifBuff.clear();
    }

//     difBuffLength=(int)((time_c-initTime_)/pro_time);//向上取整决定个数 




  }
  pressure_.push_back(cp);
  
  //内存管理，过长后去掉一些内容
  if(difBuffer.size()>10)
  {
    difBuffer.pop_front();
  }
  if(FS>1&&pressure_.size()>10*FS&&currentDataIndex_%int(FS)==0)
  {
    pressure_.erase(pressure_.begin(),pressure_.end()-10*int(FS));
  }
  if(currentDataIndex_>0.5*INT_MAX)
  {
    currentDataIndex_=0;//接近的时候从0开始
  }
  return ISCHANGEFLOOR;
}

int fous=3;
void AttitudeObserver::calculateChangeFloorType()
{
  double minKkValue=1e+5;
  double KkValue=0;
  int minKkIndex=0;
  
  
  if(abs(FS)<1e-5)
  {
    getFrequency();
  }
  
  unsigned int MaxJ=0;
  if((pressure_.size()-FS*6)>MaxJ)
  {   
    MaxJ=(int)(pressure_.size()-FS*6);
  }

  for(unsigned int j=MaxJ;j< pressure_.size()-FS;j++)
  {  
      KkValue=ISCHANGEFLOOR*(pressure_[j-FS*fous].value_flitered+pressure_[j+FS].value_flitered-2*pressure_[j].value_flitered);
      if(KkValue<minKkValue)
      {
	minKkValue=KkValue;
	minKkIndex=j;
      }   
  }
  k_temp= leastSquare(minKkIndex);
  k_temp=k_temp/0.125*FS*fous;
  
  FLOORTYPE=getChangeType3(k_temp,accChange_);
/*  cout<<k_temp<<",,"<<accChange_<<endl;
  cout<<point_count_<<"点处,时间为"<<pressure_.back().time_r-initTime_<<"发生换层，上下？"<<ISCHANGEFLOOR<<"类型是："<<FLOORTYPE<<"到达楼层为："<<currentFloorIndex<<endl;
 */ 
}
deque<double> acc_cal_buf;
double mean_acc, variance_acc;
double AttitudeObserver::leastSquare(int ind_min)
{
        double k = 0.0f;
	int x=0;
	double t1 = 0.0f, t2 = 0.0f, t3 = 0.0f, t4 = 0.0f;
	for(int j=ind_min-FS*fous-1;j<ind_min;j++)
	{
	  x++;
	  t1+=x*x;
	  t2+=x;
	  t3+=x*pressure_[j].value_flitered;
	  t4+=pressure_[j].value_flitered;
	  acc_cal_buf.push_back(pressure_[j].data_x);
	 
	}
	k=(t3*x-t2*t4)/(t1*x-t2*t2);
	
	meanStdDev(acc_cal_buf, &mean_acc,  &variance_acc,  &accChange_);
        return k;
	
}
void AttitudeObserver::calLikeAngle(int ind_min)
{
  double calsinx=0,calcosx=0,magtheta=0;
	for(int j=ind_min-FS*fous-1;j<ind_min;j++)
	{
	  calsinx+=sin(pressure_[j].data_y);
	  calcosx+=cos(pressure_[j].data_y);
	}
	calsinx/=(FS*fous);
	calcosx/=(FS*fous);
	magtheta=sqrt(calsinx*calsinx+calcosx*calcosx);
	floorHeading=acos(calcosx/magtheta);
	if(calsinx<0)
	{
	  floorHeading*=-1.0;
	}
}

int AttitudeObserver::getChangeType3(double k, double acc_std)
{
        if(abs(acc_std) > 1.0f && (abs(k) > 0.85f && abs(k) < 2.9f) ) {
            return 5;
        } else if(k < -0.3f && abs(acc_std) > 0.5f) {
            return 5;
        } else if(abs(k) > 2.4f) {
            return 3;
        } else {
            return 4;
        }
}

void AttitudeObserver::getFrequency()
{
  double deta_time=pressure_.back().time_r-pressure_[0].time_r;
  FS=(int)(pressure_.size()*1.0/deta_time);
//   cout<<FS<<endl;
}
void AttitudeObserver::addFloorHeight(double floorHeight)
{
  floorHeight_.push_back(floorHeight);
}

void AttitudeObserver::updateFloorHeightMatrix()
{
	     
              for (int i = 1; i <= floorHeight_.size()-1; i ++) {
                for (int j = 1; j <= floorHeight_.size()-i; j ++) {
                    heightMatrix_[j-1][j+i-1] = heightMatrix_[j-1][j+i-2] + floorHeight_[j+i-2];
                    heightMatrix_[j+i-1][j-1] = heightMatrix_[j+i-2][j-1] - floorHeight_[j+i-2];
                }
            }
//              cout<<"heightmatrix"<<endl;
// 	     for(int i=0;i<floorHeight_.size();i++)
// 	     {
// 	       for(int k=0;k<floorHeight_.size();k++)
// 	       {
// 		 cout<<","<<heightMatrix_[i][k];
// 	       }
// 	       cout<<";"<<endl;
// 	     }
	     
}

int AttitudeObserver::getCurrentFloorIndex()
{
  return currentFloorIndex;
}
void AttitudeObserver::setCurrentFloorIndex(int floorId)
{
  currentFloorIndex=floorId;
}

double AttitudeObserver::getAccStd()
{
  return accChange_;
}
double AttitudeObserver::getK()
{
  return k_temp;
}
int AttitudeObserver::getFloorType()
{
  return FLOORTYPE;
}
double AttitudeObserver::getCurrentFloorHeading()
{
  return floorHeading;
}


}
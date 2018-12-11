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

#ifndef ATTITUDEOBSERVER_H
#define ATTITUDEOBSERVER_H
#include "common_include.h"
namespace SHS{
class AttitudeObserver
{
public:
double FS;//采样频率FrequeOfSample
int currentFloorIndex;
int ISCHANGEFLOOR;
int FLOORTYPE;
double pro_time;
double k_temp;
double accChange_;
int addData(double accNorm,double orientation,double pressure,double time_c);
void addFloorHeight(double floorHeight);
AttitudeObserver(int floorId);
~AttitudeObserver();
vector<dataPoint > pressure_;
deque<double> difBuffer;
void updateFloorHeightMatrix();
int getCurrentFloorIndex();
double getK();
double getAccStd();
int getFloorType();


private:
double threshold;//阈值
double factor ;//楼层高度的比例
double factorBase ;//刷新基线的要求
double recordPressure_;
double recordTime_;
int point_count_;
double initTime_;
int currentDataIndex_;
vector<double> floorHeight_;
deque<double> calDifBuff;

void getFrequency();
double heightMatrix_[100][100];
void calculateChangeFloorType();
double leastSquare(int ind_min);
int getChangeType3(double k, double acc_std);

};
}
#endif // ATTITUDEOBSERVER_H

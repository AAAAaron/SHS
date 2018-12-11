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
//带地磁辅助的EKF,现在地磁可用是直接判断模量是否过大或者过小
#ifndef AHRSROBUST_H
#define AHRSROBUST_H
#include "common_include.h"
#include "ahrsangle.h"
namespace SHS
{
class AhrsRobust:public AhrsAngle
{
public:
AhrsRobust();
~AhrsRobust();
void filterUpdate(double gyro_x,double gyro_y,double gyro_z,double acc_x,double acc_y,double acc_z,double mag_x,double mag_y,double mag_z,double deta_time);
void filterInit(double acc_x,double acc_y,double acc_z,double mag_x ,double mag_y,double mag_z);
};
}
#endif // AHRSROBUST_H

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
//纯陀螺积分的方法
//纯陀螺初始化的时候要注意必须给出角度，这个可以不考虑磁偏角
#ifndef GYRO_H
#define GYRO_H
#include "common_include.h"
#include "ahrsangle.h"
namespace SHS
{
class Gyro:public AhrsAngle
{
public:
Gyro();
~Gyro();
void filterInit(double acc_x,double acc_y,double acc_z,double mag_x ,double mag_y,double mag_z);
void filterUpdate(double gyro_x,double gyro_y,double gyro_z,double acc_x,double acc_y,double acc_z,double mag_x,double mag_y,double mag_z,double deta_time);
};
}
#endif // GYRO_H

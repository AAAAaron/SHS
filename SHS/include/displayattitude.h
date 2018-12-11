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
 * 2018,12,01
 */

#ifndef DISPLAYATTITUDE_H
#define DISPLAYATTITUDE_H

#include "common_include.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/viz.hpp> 
#include <opencv2/imgproc.hpp>

namespace SHS
{
class DisplayAttitude
{
public:
cv::viz::Viz3d vis;

DisplayAttitude();
void SetCurrentFrame(cv::Affine3d& phoneAffine3d,cv::Affine3d& WorldAffine3d);
void SetStringContent(const string &text);
~DisplayAttitude();
};
}
#endif // DISPLAYATTITUDE_H

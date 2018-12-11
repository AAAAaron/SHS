/*布特沃斯滤波器，只实现了低通部分
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

#ifndef BTWFILTER_H
#define BTWFILTER_H
#include <complex>
#include <complex.h>

namespace SHS
{
class BtwFilter
{
public:
BtwFilter();
~BtwFilter();
void mybutter(int n, double Wn[], int type, int analog, double *ab, double *bb);
};
}
#endif // BTWFILTER_H

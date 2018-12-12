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
using cv::Mat;
namespace SHS
{
class DisplayAttitude
{
public:
cv::viz::Viz3d vis;
vector<POIPoint> poiVector;
int imgCount;
DisplayAttitude();
void SetCurrentFrame(cv::Affine3d& phoneAffine3d,cv::Affine3d& WorldAffine3d);
void SetStringContent(const string &text);
void ProcessImgAr(string fileName,Eigen::Vector3d &current_phone,Eigen::Quaterniond& Qpw);
~DisplayAttitude();

static bool AddPoi(Mat img_1,const Eigen::Vector3d &current_phone,const Eigen::Quaterniond &Qpw,const Eigen::Vector3d &poi_vec,CvPoint &cp)
{
  
    Eigen::Isometry3d Tcp=Eigen::Isometry3d::Identity();                // 从设备坐标系到相机坐标系
    Eigen::Isometry3d Tpw=Eigen::Isometry3d::Identity();                // 从世界坐标系到设备坐标系
    
    Eigen::Quaterniond Qcp=angle2quat(M_PI/2,M_PI,0);
    Tcp.rotate ( Eigen::AngleAxisd(Qcp) );                                     // 按照rotation_vector进行旋转
    Tcp.pretranslate ( Eigen::Vector3d ( 0.0680,0.0325,0 ) );                     // 这个是设备到相机的平移，和手机有关
    Tpw.rotate(Qpw);
  
    Eigen::Vector3d poi_c = Tpw*(poi_vec-current_phone);// 在手机坐标系上的位置,由于current_phone是在世界坐标系下的位置，所以要先减去
//     cout<<"在手机坐标系上的位置"<<endl;
//     cout<<poi_c<<endl;
    poi_c=Tcp*poi_c;
//     poi_c=Tcp*(poi_c-Eigen::Vector3d ( 0.0325,0.0680,0 ));// 在相机坐标系上的位置
//     cout<<"在相机坐标系上的位置"<<endl;
//     cout<<poi_c<<endl;
    int pixel_length=img_1.cols;
    int pixel_width=img_1.rows;
    double cmos_length=6.69;
    double cmos_width=5.21;
    double f=4.0;//焦距mm
//     cout<<poi_c(2)<<endl;
    if(poi_c(2)<=0)
    {
      return false;
    }
    Mat pc=(cv::Mat_<double>(3,1)<<poi_c(0)/poi_c(2),poi_c(1)/poi_c(2),1);//归一化像素平面 
    Mat K=(cv::Mat_<double>(3,3)<<f*pixel_length/cmos_length,0,pixel_length/2,0,f*pixel_width/cmos_width,pixel_width/2,0,0,1);
    Mat puv=(cv::Mat_<double>(3,1)<<0,0,0);
    puv=K*pc;
    cp.x=(int)puv.at<double>(0,0);
    cp.y=(int)puv.at<double>(1,0);//opencv调用的问题，互相交换xy的位置以及uv
    return true;
}

static bool AddDirectionStarff(Mat img_1)
{
      int pixel_length=img_1.cols;
      int pixel_width=img_1.rows;
      
      CvPoint cpstart,cpend;
      cpstart.x=4/16.0*pixel_length;
      cpstart.y=0;
      cpend.x=12/16.0*pixel_length;
      cpend.y=15;
      cv::rectangle(img_1,cpstart,cpend,cv::Scalar(50, 255, 50));
      for(int i=8;i<=24;i++)
      {
	cpstart.x=i/32.0*pixel_length;
	cpstart.y=0;
	cpend.x=i/32.0*pixel_length;
	cpend.y=15;
	cv::line(img_1,cpstart,cpend,cv::Scalar(255,255,255),1);
	if(i==16)
	{
	  cpend.x-=5;
	  cpend.y+=10;
	  cv::putText(img_1,"East",cpend,1,0.4, cv::Scalar(255, 255, 255), 0.5, 8);	//BGR
	}
      }
      
}

static bool AddArrowFig(Mat img_1)
{
  Mat arrow=cv::imread("../media/arrow_meitu_4.jpg");
//   resize(arrow,arrow,Size(arrow.cols/2,arrow.rows/2),0,0,INTER_LINEAR);
  Mat imagePOI = img_1(cv::Rect(img_1.cols/2.0-arrow.cols/2.0,img_1.rows-arrow.rows,arrow.cols,arrow.rows));
  Mat mask = cv::imread("../media/arrow_meitu_4.jpg",0);
  arrow.copyTo(imagePOI,mask);
}

static void AddArrowLine(Mat img_1,CvPoint &cpstart,CvPoint &cpend)
{
/*
 * 箭头形状为..............cpend
 * ......................         .....
 * ..cpL21_1...cpL22_1                cpL22_2.....cpL21_2...
 * ...............                       ................
 * ...............cpL23_1             cpL23_2............
 * ..........................................
  
  */
  double scale_L21=0.7;
  
  
  double rotate_angle1=M_PI/3.0;
  double rotate_angle2=0.5*rotate_angle1;
  double rotate_angle3=0.5*rotate_angle2;
  
  double scale_L22=1.2*scale_L21*cos(rotate_angle1)/cos(rotate_angle2);//内侧线长比例
  cv::Point cpL21_1,cpL21_2,cpL22_1,cpL22_2,cpL23_1,cpL23_2;
  Eigen::Vector3d L21 (cpstart.x-cpend.x,cpstart.y-cpend.y,0);
  Eigen::AngleAxisd arrow_rotate(rotate_angle1,Eigen::Vector3d ( 0,0,1 ));
  Eigen::Vector3d L21_1 = arrow_rotate*L21;
  L21_1*=scale_L21;
  cpL21_1.x=cpend.x+L21_1(0);
  cpL21_1.y=cpend.y+L21_1(1);
  arrow_rotate=Eigen::AngleAxisd(-rotate_angle1,Eigen::Vector3d ( 0,0,1 ));
  Eigen::Vector3d L21_2=arrow_rotate*L21;
  L21_2*=scale_L21;
  cpL21_2.x=cpend.x+int(round(L21_2(0)));
  cpL21_2.y=cpend.y+int(round(L21_2(1)));
  
  arrow_rotate=Eigen::AngleAxisd(rotate_angle2,Eigen::Vector3d ( 0,0,1 ));
  Eigen::Vector3d L22_1=arrow_rotate*L21;
  L22_1*=scale_L22;
  cpL22_1.x=cpend.x+int(round(L22_1(0)));
  cpL22_1.y=cpend.y+int(round(L22_1(1))); 
  arrow_rotate=Eigen::AngleAxisd(-rotate_angle2,Eigen::Vector3d ( 0,0,1 ));
  Eigen::Vector3d L22_2=arrow_rotate*L21;
  L22_2*=scale_L22;
  cpL22_2.x=cpend.x+int(round(L22_2(0)));
  cpL22_2.y=cpend.y+int(round(L22_2(1)));
  
  
  arrow_rotate=Eigen::AngleAxisd(rotate_angle3,Eigen::Vector3d ( 0,0,1 ));
  Eigen::Vector3d L23_1=arrow_rotate*L21;
  cpL23_1.x=cpend.x+int(round(L23_1(0)));
  cpL23_1.y=cpend.y+int(round(L23_1(1)));

  arrow_rotate=Eigen::AngleAxisd(-rotate_angle3,Eigen::Vector3d ( 0,0,1 ));
  Eigen::Vector3d L23_2=arrow_rotate*L21;
  cpL23_2.x=cpend.x+int(round(L23_2(0)));
  cpL23_2.y=cpend.y+int(round(L23_2(1)));  

  
  cv::Point* points=new cv::Point[8];
  points[0]=cpend;
  points[1]=cpL21_1;
  points[2]=cpL22_1;
  points[3]=cpL23_1;
  points[4]=cpL23_2;
  points[5]=cpL22_2;
  points[6]=cpL21_2;
  points[7]=cpend;
//   points[2]=cpL23_1;
//   points[3]=cpL23_2;
//   points[4]=cpL21_2;
//   points[5]=cpend;

  
  
//   for(int i=0;i<6;i++)
//   {
//     cv::putText(img_1,to_string(i),points[i],FONT_HERSHEY_SIMPLEX,1, Scalar(0, 0, 255));
//   }
  
  
//   cv::fillConvexPoly(img_1,points,6,Scalar(0, 0, 255));
  cv::fillConvexPoly(img_1,points,8,cv::Scalar(0, 0, 255));
  
  delete[] points;
  
  
//   cv::line(img_1,cpend,cpL21_1,Scalar(255, 255, 255));
//   cv::line(img_1,cpend,cpL21_2,Scalar(255, 255, 255));
//   cv::line(img_1,cpstart,cpend,Scalar(0, 0, 255),10);
  
  
  
}

static bool AddMAp(Mat img_1,const Eigen::Vector3d &current_phone)
{
  Mat arrow=cv::imread("../media/map_meitu_2.jpg");
//   resize(arrow,arrow,Size(arrow.cols/2,arrow.rows/2),0,0,INTER_LINEAR);
  Mat imagePOI = img_1(cv::Rect(img_1.cols-arrow.cols-3,img_1.rows-arrow.rows-3,arrow.cols,arrow.rows));
  Mat mask = cv::imread("../media/map_meitu_2.jpg",0);
  arrow.copyTo(imagePOI,mask);
  cv::Point cp;
  cp.x=-current_phone(0)/80*arrow.cols+img_1.cols-3;
  cp.y=-current_phone(1)/80*arrow.rows+img_1.rows-3;
  cv::circle(img_1,cp,3,cv::Scalar(255, 0, 0),2);
}

static double hfromdis(double distance)
{
  return (16/(1+exp((0.2*(8-distance))))-8)*0.5;
}
};



}
#endif // DISPLAYATTITUDE_H

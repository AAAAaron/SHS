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

#include "displayattitude.h"
namespace SHS{
  cv::viz::WText3D* attContent;
// cv::viz::WText3D atttest;
      vector<int> compression_params;

DisplayAttitude::DisplayAttitude()
{
    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);  //选择jpeg
    compression_params.push_back(100); //在这个填入你要的图片质量
    
    
    vis=cv::viz::Viz3d("Visual Odometry");
    cv::viz::WCoordinateSystem world_coor(1.0), camera_coor(0.5);
    
//     cv::viz::WPlane phone(cv::Size2d(0.5,1),cv::viz::Color::silver());
    cv::Mat img_2;
    img_2= cv::imread ("../media/phone_meitu_1.jpg", CV_LOAD_IMAGE_COLOR );
    double camra_length=5;
    cv::viz::WImage3D phone(img_2,cv::Size2d(0.5,1));
    cv::Point3d textpos(1,1,1) ,cam_pos( camra_length,-camra_length, camra_length), cam_focal_point(0,camra_length/2.0,0), cam_y_dir(0,0,-1);
//     attContent=cv::viz::WText3D("ATT",textpos); 
//     cv::viz::WText3D atttest("ATT",textpos,0.2,true,cv::viz::Color::white());
    attContent=new cv::viz::WText3D("ATT",textpos,0.1,true,cv::viz::Color::cyan());
    
    
    cv::viz::WText3D X("EAST",cv::Point3d(1,0,0),0.1,true,cv::viz::Color::red());
    cv::viz::WText3D Y("NORTH",cv::Point3d(0,1,0),0.1,true,cv::viz::Color::green());
    cv::viz::WText3D Z("SKY",cv::Point3d(0,0,1),0.1,true,cv::viz::Color::blue());
    
    cv::Affine3d cam_pose = cv::viz::makeCameraPose( cam_pos, cam_focal_point, cam_y_dir );
    vis.setViewerPose( cam_pose );
    
    world_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
//     camera_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);

    vis.showWidget( "World", world_coor );
    vis.showWidget( "Phone", phone );
    vis.showWidget( "Camera", camera_coor );
    vis.showWidget( "attc", *attContent );
    
    vis.showWidget("x",X);
    vis.showWidget("y",Y);
    vis.showWidget("z",Z);
    imgCount=0;
}
void DisplayAttitude::SetCurrentFrame(cv::Affine3d& phoneAffine3d, cv::Affine3d& WorldAffine3d)
{
  

//   	cv::waitKey(5);
	
	vis.setWidgetPose( "World", WorldAffine3d);
        vis.setWidgetPose( "Camera", phoneAffine3d);
	vis.setWidgetPose( "Phone", phoneAffine3d);
        vis.spinOnce(10, false);
}
void DisplayAttitude::SetStringContent(const string& text)
{
  (*attContent).setText(text);
//   (*attContent).getText();
}

DisplayAttitude::~DisplayAttitude()
{
  
}
Eigen::Vector3d poi(0,0,0);
CvPoint cp;

void DisplayAttitude::ProcessImgAr(string fileName, Eigen::Vector3d& current_phone, Eigen::Quaterniond& Qpw)
{
  imgCount++;
  Mat img_1= cv::imread (fileName, CV_LOAD_IMAGE_COLOR );
  AddDirectionStarff(img_1);
  AddMAp(img_1,current_phone);   
  for(unsigned int i=0;i<poiVector.size();i++)
  { 
    poi(0)=poiVector[i].x;
    poi(1)=poiVector[i].y;//27.16,8.566
    poi(2)=hfromdis(sqrt((current_phone(0)-poi(0))*(current_phone(0)-poi(0))+(current_phone(1)-poi(1))*(current_phone(1)-poi(1))));
    bool cs=AddPoi(img_1,current_phone,Qpw,poi,cp);
    if(cs){
	if(strcmp(poiVector[i].name.c_str(),"fridge")==0)
	{
	  CvPoint arrowstart,arrowend;
	  arrowstart.x=img_1.cols/2.0;
	  arrowstart.y=img_1.rows;
	  arrowend.x=(cp.x-arrowstart.x)/5.0+arrowstart.x;
	  arrowend.y=(cp.y-arrowstart.y)/5.0+arrowstart.y;
	  AddArrowLine(img_1,arrowstart,arrowend);
// 	  cv::line(img_1,arrowstart,arrowend,Scalar(255, 255, 255),3);  
// 	  cv::circle(img_1,arrowend,10,Scalar(0, 0, 255),1);
	  cv::putText(img_1,poiVector[i].name,cp,1,1, cv::Scalar(0, 0, 255), 2, 8);	
// 	  cvFillPoly(img_1, point1, 3, 2, color);
	}
	else{
	  cv::putText(img_1,poiVector[i].name,cp,1,0.5, cv::Scalar(255, 255, 50), 2, 8);	
	}
	 
    }
    else
    {
      continue;
    }
  }
  cv::imwrite("../img/"+to_string(imgCount)+".jpg",img_1,compression_params);

}

}
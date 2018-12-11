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
DisplayAttitude::DisplayAttitude()
{
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
    attContent=new cv::viz::WText3D("ATT",textpos,0.2,true,cv::viz::Color::white());
    
    cv::Affine3d cam_pose = cv::viz::makeCameraPose( cam_pos, cam_focal_point, cam_y_dir );
    vis.setViewerPose( cam_pose );
    
    world_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
//     camera_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);

    vis.showWidget( "World", world_coor );
    vis.showWidget( "Phone", phone );
    vis.showWidget( "Camera", camera_coor );
    vis.showWidget( "attc", *attContent );
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
}
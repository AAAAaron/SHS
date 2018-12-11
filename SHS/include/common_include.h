#ifndef COMMON_INCLUDE_H_
#define COMMON_INCLUDE_H_

#define Gravity 9.806


// for Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
// using Eigen::Vector2d;
// using Eigen::Vector3d;
// using namespace Eigen;

// for cv
// #include <opencv2/core/core.hpp>
// using cv::Mat;


#include <iostream>
using namespace std;
#include <string>
#include <vector>
#include <queue> 
#include <math.h>
#include <sstream>
#include <vector>
#include <algorithm>

#include <memory>
// #include <boost/shared_ptr.hpp>

// using namespace boost;
// #include <shared_ptr.hpp>

//for read file
#include <unistd.h>  
#include <dirent.h>  

struct dataPoint
{
	double data_x;
	double data_y;
	double data_z;
	double time_r;
	double magnitude;
	int detection_binary;
	double value_flitered;
	bool _IsStep;

};

struct PDRPoint
{
	double x;
	double y;
	double z;
	double yaw;
	double sl;
	double floor_id;
	double deta_angle;
};
enum EulerTransformOrder
{
	ZYX,ZYZ,ZXY,ZXZ,YXZ,YXY,YZX,YZY,XYZ,XYX,XZY,XZX
};
enum CalAngelMethod{
  GYRO,AHRSROBUST};//角度方法的选择
enum OutYaw{
  YAW=0,INTEGRAL=1,INTERANGLE=2};//yaw角，积分角，和北的夹角
  
template<typename _Tp>  
int meanStdDev(const deque<_Tp>& mat, double* mean, double* variance, double* stddev)  
{  
    int h = mat.size(); 
    double sum=0.;
    double sqsum= 0. ;  
  
    for (int y = 0; y < h; ++y) {  
        
        double v = static_cast<double>(mat[y]);  
        sum += v;  
        sqsum += v * v;  
         
    }  
  
    double scale = 1.0 / h;  
    *mean = sum * scale;  
    *variance = std::max(sqsum*scale - (*mean)*(*mean), 0.);  
    *stddev = std::sqrt(*variance);  
  
    return 0;  
}  

//10.22验证以下四个方法均有效,Eigen顺序是x,y,z,w
static Eigen::Vector3d threeaxisrot(double r11,double r12,double r21,double r31,double  r32){
// find angles for rotations about X, Y, and Z axes
        double r1 = atan2( r11, r12 );
        double r2 = asin( r21 );
        double r3 = atan2( r31, r32 );
		Eigen::Vector3d a;
		a<<r1,r2,r3;
		return a;
}

static Eigen::Vector3d twoaxisrot(double r11,double r12,double r21,double r31,double  r32){
        double r1 = atan2( r11, r12 );
        double r2 = acos( r21 );
        double r3 = atan2( r31, r32 );
		Eigen::Vector3d a;
		a<<r1,r2,r3;
		return a;
}
//this->quaternion=angle2quat(Att(2),Att(1),Att(0));要按照zyx顺序进，显示是xyz,现在在卡尔曼那个部分定义是yaw,pitch,row，在手机里是yaw,roll,pitch,逆时针为正
static Eigen::Quaterniond angle2quat(double r1,double r2,double r3,EulerTransformOrder modename=ZYX)
{
	Eigen::Vector4d qtmp;
	double r[3]={r1,r2,r3};
	double cang[3]={0,0,0};
	double sang[3]={0,0,0};

	for (int i = 0; i < 3; i++)
	{
		cang[i]=cos(r[i]/2.0);
		sang[i]=sin(r[i]/2.0);
	}

	switch (modename)
	{
	case ZYX://顺序都专门调过了，调的时候要仔细
			qtmp<<cang[0]*cang[1]*sang[2] - sang[0]*sang[1]*cang[2],
			cang[0]*sang[1]*cang[2] + sang[0]*cang[1]*sang[2],
			sang[0]*cang[1]*cang[2] - cang[0]*sang[1]*sang[2],
			cang[0]*cang[1]*cang[2] + sang[0]*sang[1]*sang[2];
			 
	break;
	default:
		break;
	}
	return Eigen::Quaterniond(qtmp);
}

static Eigen::Vector3d quat2angle(const Eigen::Quaterniond &q , EulerTransformOrder modename=ZYX)
{
	Eigen::Vector3d eulerAngle;
	Eigen::Quaterniond qin=Eigen::Quaterniond(q);
	qin.normalize();
	double tmp;
	switch (modename)
	{

	case ZYX:
        eulerAngle = threeaxisrot( 2*(qin.x()*qin.y() + qin.w()*qin.z()), 
                                   pow(qin.w(),2) + pow(qin.x(),2) - pow(qin.y(),2) - pow(qin.z(),2), 
                                  -2*(qin.x()*qin.z() - qin.w()*qin.y()), 
                                   2*(qin.y()*qin.z() + qin.w()*qin.x()), 
                                   pow(qin.w(),2) - pow(qin.x(),2) - pow(qin.y(),2) + pow(qin.z(),2));
	//虽然计算顺序是对的，但是为了和matlab对应，调整了显示顺序为xyz
	tmp=eulerAngle(2);
	eulerAngle(2)=eulerAngle(0);
	eulerAngle(0)=tmp;
	break;
	case ZYZ:
        eulerAngle = twoaxisrot( 2*(qin.y()*qin.z() - qin.w()*qin.x()), 
                                 2*(qin.x()*qin.z() + qin.w()*qin.y()), 
                                 pow(qin.w(),2) - pow(qin.x(),2) - pow(qin.y(),2) + pow(qin.z(),2), 
                                 2*(qin.y()*qin.z() + qin.w()*qin.x()), 
                                -2*(qin.x()*qin.z() - qin.w()*qin.y()));
    break;            
	case ZXY:
        eulerAngle = threeaxisrot( -2*(qin.x()*qin.y() - qin.w()*qin.z()), 
                                    pow(qin.w(),2) - pow(qin.x(),2) + pow(qin.y(),2) - pow(qin.z(),2), 
                                    2*(qin.y()*qin.z() + qin.w()*qin.x()), 
                                   -2*(qin.x()*qin.z() - qin.w()*qin.y()), 
                                    pow(qin.w(),2) - pow(qin.x(),2) - pow(qin.y(),2) + pow(qin.z(),2));
	break;
    case ZXZ:
       eulerAngle = twoaxisrot( 2*(qin.x()*qin.z() + qin.w()*qin.y()), 
                                -2*(qin.y()*qin.z() - qin.w()*qin.x()), 
                                 pow(qin.w(),2) - pow(qin.x(),2) - pow(qin.y(),2) + pow(qin.z(),2), 
                                 2*(qin.x()*qin.z() - qin.w()*qin.y()), 
                                 2*(qin.y()*qin.z() + qin.w()*qin.x()));
	break;
	case YXZ:
        eulerAngle = threeaxisrot( 2*(qin.x()*qin.z() + qin.w()*qin.y()), 
                                   pow(qin.w(),2) - pow(qin.x(),2) - pow(qin.y(),2) + pow(qin.z(),2), 
                                  -2*(qin.y()*qin.z() - qin.w()*qin.x()), 
                                   2*(qin.x()*qin.y() + qin.w()*qin.z()), 
                                   pow(qin.w(),2) - pow(qin.x(),2) + pow(qin.y(),2) - pow(qin.z(),2));
	break;       
	case YXY:
        eulerAngle = twoaxisrot( 2*(qin.x()*qin.y() - qin.w()*qin.z()), 
                                 2*(qin.y()*qin.z() + qin.w()*qin.x()), 
                                 pow(qin.w(),2) - pow(qin.x(),2) + pow(qin.y(),2) - pow(qin.z(),2), 
                                 2*(qin.x()*qin.y() + qin.w()*qin.z()), 
                                -2*(qin.y()*qin.z() - qin.w()*qin.x()));
	break;      
    case YZX:       
        eulerAngle = threeaxisrot( -2*(qin.x()*qin.z() - qin.w()*qin.y()), 
                                    pow(qin.w(),2) + pow(qin.x(),2) - pow(qin.y(),2) - pow(qin.z(),2), 
                                    2*(qin.x()*qin.y() + qin.w()*qin.z()), 
                                   -2*(qin.y()*qin.z() - qin.w()*qin.x()), 
                                    pow(qin.w(),2) - pow(qin.x(),2) + pow(qin.y(),2) - pow(qin.z(),2));
	break;        
	case YZY:
        eulerAngle = twoaxisrot( 2*(qin.y()*qin.z() + qin.w()*qin.x()), 
                                -2*(qin.x()*qin.y() - qin.w()*qin.z()), 
                                 pow(qin.w(),2) - pow(qin.x(),2) + pow(qin.y(),2) - pow(qin.z(),2), 
                                 2*(qin.y()*qin.z() - qin.w()*qin.x()), 
                                 2*(qin.x()*qin.y() + qin.w()*qin.z()));
	break;
    case XYZ:
        eulerAngle = threeaxisrot( -2*(qin.y()*qin.z() - qin.w()*qin.x()), 
                                    pow(qin.w(),2) - pow(qin.x(),2) - pow(qin.y(),2) + pow(qin.z(),2), 
                                    2*(qin.x()*qin.z() + qin.w()*qin.y()), 
                                   -2*(qin.x()*qin.y() - qin.w()*qin.z()), 
                                    pow(qin.w(),2) + pow(qin.x(),2) - pow(qin.y(),2) - pow(qin.z(),2));
	break;        
    case XYX:
        eulerAngle = twoaxisrot( 2*(qin.x()*qin.y() + qin.w()*qin.z()), 
                                -2*(qin.x()*qin.z() - qin.w()*qin.y()), 
                                 pow(qin.w(),2) + pow(qin.x(),2) - pow(qin.y(),2) - pow(qin.z(),2), 
                                 2*(qin.x()*qin.y() - qin.w()*qin.z()), 
                                 2*(qin.x()*qin.z() + qin.w()*qin.y()));
	break;        
    case XZY:
        eulerAngle = threeaxisrot( 2*(qin.y()*qin.z() + qin.w()*qin.x()), 
                                   pow(qin.w(),2) - pow(qin.x(),2) + pow(qin.y(),2) - pow(qin.z(),2), 
                                  -2*(qin.x()*qin.y() - qin.w()*qin.z()), 
                                   2*(qin.x()*qin.z() + qin.w()*qin.y()), 
                                   pow(qin.w(),2) + pow(qin.x(),2) - pow(qin.y(),2) - pow(qin.z(),2));
	break;        
    case XZX:
        eulerAngle = twoaxisrot( 2*(qin.x()*qin.z() - qin.w()*qin.y()), 
                                 2*(qin.x()*qin.y() + qin.w()*qin.z()), 
                                 pow(qin.w(),2) + pow(qin.x(),2) - pow(qin.y(),2) - pow(qin.z(),2), 
                                 2*(qin.x()*qin.z() + qin.w()*qin.y()), 
                                -2*(qin.x()*qin.y() - qin.w()*qin.z()));

	default:
		break;
	}	
	return eulerAngle;

}

static Eigen::Matrix<double ,3,4> jacobianES(const Eigen::Quaterniond &q,const Eigen::Vector3d &v,string form)
{
	
	Eigen::Matrix<double ,3,4> H=Eigen::Matrix<double ,3,4>::Zero();
	if (strcmp(form.c_str(),"long")==0)
	{
		H << 2*q.w()*v(0) - 2*q.y()*v(2) + 2*q.z()*v(1), 2*q.x()*v(0) + 2*q.y()*v(1) + 2*q.z()*v(2), 2*q.x()*v(1) - 2*q.w()*v(2) - 2*q.y()*v(0), 2*q.w()*v(1) + 2*q.x()*v(2) - 2*q.z()*v(0),
			 2*q.w()*v(1) + 2*q.x()*v(2) - 2*q.z()*v(0), 2*q.w()*v(2) - 2*q.x()*v(1) + 2*q.y()*v(0), 2*q.x()*v(0) + 2*q.y()*v(1) + 2*q.z()*v(2), 2*q.y()*v(2) - 2*q.w()*v(0) - 2*q.z()*v(1),
			 2*q.w()*v(2) - 2*q.x()*v(1) + 2*q.y()*v(0), 2*q.z()*v(0) - 2*q.x()*v(2) - 2*q.w()*v(1), 2*q.w()*v(0) - 2*q.y()*v(2) + 2*q.z()*v(1), 2*q.x()*v(0) + 2*q.y()*v(1) + 2*q.z()*v(2);

	}
	else
	{
		if (strcmp(form.c_str(),"short")==0)
		{
			H <<2*q.z()*v(1) - 2*q.y()*v(2),           2*q.y()*v(1) + 2*q.z()*v(2), 2*q.x()*v(1) - 2*q.w()*v(2) - 4*q.y()*v(0), 2*q.w()*v(1) + 2*q.x()*v(2) - 4*q.z()*v(0) ,
					2*q.x()*v(2) - 2*q.z()*v(0), 2*q.w()*v(2) - 4*q.x()*v(1) + 2*q.y()*v(0),           2*q.x()*v(0) + 2*q.z()*v(2), 2*q.y()*v(2) - 2*q.w()*v(0) - 4*q.z()*v(1) ,
					2*q.y()*v(0) - 2*q.x()*v(1), 2*q.z()*v(0) - 4*q.x()*v(2) - 2*q.w()*v(1), 2*q.w()*v(0) - 4*q.y()*v(2) + 2*q.z()*v(1),           2*q.x()*v(0) + 2*q.y()*v(1) ;


		}
		else
		{
			cout<<"Not a known form (short or long)"<<form<<endl;
			// throw form;
			
		}
	}
	return H;
}

static Eigen::Vector3d quatrotate(const Eigen::Quaterniond &q, const Eigen::Vector3d & v,string form="short"){


	Eigen::Vector3d vo(0,0,0);

	if (strcmp(form.c_str(),"long")==0){
	vo << 	v(0)*(q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z()) + v(1)*(2*q.w()*q.z() + 2*q.x()*q.y()) - v(2)*(2*q.w()*q.y() - 2*q.x()*q.z()) ,
		v(1)*(q.w()*q.w()- q.x()*q.x() + q.y()*q.y() - q.z()*q.z()) - v(0)*(2*q.w()*q.z() - 2*q.x()*q.y()) + v(2)*(2*q.w()*q.x() + 2*q.y()*q.z()) ,
		v(2)*(q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z()) + v(0)*(2*q.w()*q.y() + 2*q.x()*q.z()) - v(1)*(2*q.w()*q.x() - 2*q.y()*q.z());
	}
	else{
	  if (strcmp(form.c_str(),"short")==0)
	  {
	    vo <<v(1)*(2*q.w()*q.z() + 2*q.x()*q.y()) - v(0)*(2*q.y()*q.y() + 2*q.z()*q.z() - 1) - v(2)*(2*q.w()*q.y() - 2*q.x()*q.z()) ,
		v(2)*(2*q.w()*q.x() + 2*q.y()*q.z()) - v(0)*(2*q.w()*q.z() - 2*q.x()*q.y()) - v(1)*(2*q.x()*q.x() + 2*q.z()*q.z() - 1) ,
		v(0)*(2*q.w()*q.y() + 2*q.x()*q.z()) - v(2)*(2*q.x()*q.x() + 2*q.y()*q.y() - 1)- v(1)*(2*q.w()*q.x() - 2*q.y()*q.z());
	    
	  }
	  else
	  {
	    cout<<"Not a known form (short or long)"<<endl;
	    
	  }
	}
	return vo;
}

static Eigen::Quaterniond para2quat(double gx,double gy,double gz,double mx,double my,double mz)
{

//     double pitch = asin(gx/Gravity);  //  俯仰
//     double roll = -asin(gy/Gravity*cos(pitch));  //  横滚
//             // 磁力计确定偏航角
//     double Mx = mx*cos(pitch) + my*sin(roll)*sin(pitch) + mz*cos(roll)*sin(pitch);
//     double My =my*cos(roll) - mz*sin(roll);
//     double yaw = -atan2(My,Mx); 
//     return angle2quat(yaw,pitch,roll);
    
    
    double pitch = asin(gy/Gravity);  //  俯仰
    double roll = -asin(gx/Gravity*cos(pitch));  //  横滚
            // 磁力计确定偏航角
    double Mx = my*cos(pitch) + mx*sin(roll)*sin(pitch) -mz*cos(roll)*sin(pitch);
    double My =mx*cos(roll) + mz*sin(roll);
    double yaw = -atan2(My,Mx); 
//     return angle2quat(yaw,roll,pitch);//如果按照ahars那个换了xyz顺序的话，使用这个顺序和EIGEN四元素输出的旋转矩阵就一样了
    return angle2quat(yaw,pitch,roll);
}

static vector<string> getFiles(string cate_dir)  
{  
    vector<string> files;
    

  
    DIR *dir;  
    struct dirent *ptr;  


    if ((dir=opendir(cate_dir.c_str())) == NULL)  
        {  
	  cout<<"opendir is wrong"<<endl;
        }  
    while ((ptr=readdir(dir)) != NULL)  
    {  
        if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    ///current dir OR parrent dir  
                continue;  
        else{ 
	  switch(ptr->d_type){
	    case 8:///file  
	    {
// 	      printf("d_name:%s/%s\n",basePath,ptr->d_name); 
	      cout<<ptr->d_name<<endl;
	      files.push_back(cate_dir+"/"+ptr->d_name); 
	      break;
	    }
	    case 10:    ///link file  
	    {  //printf("d_name:%s/%s\n",basePath,ptr->d_name);  
	      continue;
	      break;	  
	    }
	    case 4:    ///dir    
	    {  
	      vector<string> tempfiles;
	      tempfiles=getFiles(cate_dir+"/"+ptr->d_name);
	      for(unsigned int i=0;i<tempfiles.size();i++)
	      {
		files.push_back(tempfiles[i]); 
	      }

	      break;
	    }  
	    default:
	      continue;
	      
	  }
	  
	}
	     
         
    }  
    closedir(dir);  

    return files;
} 

#endif
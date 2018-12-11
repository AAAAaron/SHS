
#include <fstream>
#include "common_include.h"
#include "displayattitude.h"

int main(int argc, char** argv){
    SHS::DisplayAttitude dst;
	Eigen::Quaterniond test=angle2quat(M_PI/2,M_PI/3,M_PI/4);//如果用这个函数的话，实际输入要求是yaw,roll,pitch
	cout<<test.coeffs()<<endl;
	cout<<"----"<<endl;
	cout<<test.toRotationMatrix()<<endl;
	test=test.conjugate();
	cout<<test.coeffs()<<endl;
	cout<<"----"<<endl;
	cout<<test.toRotationMatrix()<<endl;	
	Eigen::Quaterniond worldq=angle2quat(0,0,0);
	
 	cv::Affine3d PhoneAtt(
            cv::Affine3d::Mat3( test.toRotationMatrix()(0,0),test.toRotationMatrix()(0,1),test.toRotationMatrix()(0,2),
				test.toRotationMatrix()(1,0),test.toRotationMatrix()(1,1),test.toRotationMatrix()(1,2),
				test.toRotationMatrix()(2,0),test.toRotationMatrix()(2,1),test.toRotationMatrix()(2,2)
            ), 
            cv::Affine3d::Vec3(
                2,3,0
            )
        );
	
	cv::Affine3d World(
            cv::Affine3d::Mat3( worldq.toRotationMatrix()(0,0),worldq.toRotationMatrix()(0,1),worldq.toRotationMatrix()(0,2),
				worldq.toRotationMatrix()(1,0),worldq.toRotationMatrix()(1,1),worldq.toRotationMatrix()(1,2),
				worldq.toRotationMatrix()(2,0),worldq.toRotationMatrix()(2,1),worldq.toRotationMatrix()(2,2)
            ), 
            cv::Affine3d::Vec3(
                0,0,0
            )
        );   
    for(int i=1;i<1000;i++){
      dst.SetCurrentFrame(PhoneAtt,World);
      getchar();
    }
  
    return 0;
  
  
}
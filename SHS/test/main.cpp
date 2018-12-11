// SHS_pro.cpp : 定义控制台应用程序的入口点。
//

#include <fstream>
#include <time.h>
#include "common_include.h"
#include "pdrsim.h"
#include "displayattitude.h"
#include "config.h"
void OnPdrStepCallbackEvent(double  x, double y, double sl, double yaw ,double deta_angle)
{
//      cout<<x<<","<<y<<","<<","<<sl<<","<<yaw<<","<<deta_angle<<endl;
}

int main(int argc, char** argv)
{
  cout<<"-------"<<endl;
    SHS::DisplayAttitude dst;
	Eigen::Quaterniond test=angle2quat(M_PI/2,0,0);//如果用这个函数的话，实际输入要求是yaw,roll,
	Eigen::Quaterniond worldq=angle2quat(0,0,0);
    SHS::Config::setParameterFile ("../config/default.yaml");
    
	time_t start,stop;
	start = time(NULL);

	SHS::PDRSIM _ptest;
	_ptest.stest->set_para(1.3,0.9);
	int mode=SHS::Config::get<int> ( "choose_method" );
	double setyaw=SHS::Config::get<double> ( "init_yaw" );
	double BJoffset=-7.0*M_PI/180.0;
	_ptest.pdrEkf=bool( SHS::Config::get<int> ( "pdrEkf" ));
	double initx=SHS::Config::get<double> ( "init_x" );
	double inity=SHS::Config::get<double> ( "init_y" );

// 	istringstream( SHS::Config::get<string> ( "pdrEkf" ))>>_ptest.pdrEkf;
	cout<<"使用ekf?:"<<_ptest.pdrEkf<<endl;
	switch(mode)
	{
	  //积分角度，积分转角
	  case 1:{
	    	_ptest.choose_ahrs(1);
		_ptest.YAWORINTE=OutYaw(1);
		_ptest.InitialXYYaw(initx,inity,setyaw);
		
	    break;
	  }
	  //积分角度，欧拉角
	  case 2:{
	    	_ptest.choose_ahrs(1);
		_ptest.YAWORINTE=OutYaw(0);
		_ptest.InitialXYYaw(initx,inity,setyaw);
	    break;	    
	  }
	  //积分角度，夹角
	  case 3:{
	    	_ptest.choose_ahrs(1);
		_ptest.YAWORINTE=OutYaw(2);
		_ptest.InitialXYYaw(initx,inity,setyaw);
	    break;	    
	  }
	  //鲁邦角度，欧拉角
	  case 4:{
	    	_ptest.choose_ahrs(2);
		_ptest.YAWORINTE=OutYaw(0);
		_ptest.InitialXY(initx,inity);
 		_ptest.set_magoffset(BJoffset);//北京磁偏北偏东7°，南特大约为北偏西0.39
	    break;	    
	  }
	  //鲁邦角度，积分角度
	  case 5:{
	    	_ptest.choose_ahrs(2);
		_ptest.YAWORINTE=OutYaw(1);
// 		_ptest.InitialXY(initx,inity);
		_ptest.InitialXYYaw(initx,inity,setyaw);
		_ptest.set_magoffset(BJoffset);//北京磁偏北偏东7°，南特大约为北偏西0.39
	    break;	    
	  }	
	  case 6:{
	    	_ptest.choose_ahrs(2);
		_ptest.YAWORINTE=OutYaw(2);
		_ptest.InitialXY(initx,inity);
		_ptest.set_magoffset(BJoffset);//北京磁偏北偏东7°，南特大约为北偏西0.39
	    break;	    
	  }	
	  //ahrsqsmf yaw
	  case 7:{
	    	_ptest.choose_ahrs(3);
		_ptest.YAWORINTE=OutYaw(0);
		_ptest.InitialXY(initx,inity);
		_ptest.set_magoffset(BJoffset);//北京磁偏北偏东7°，南特大约为北偏西0.39
	    break;	    
	  }
	  //ahrsqsmf inte
	  case 8:{
	    	_ptest.choose_ahrs(3);
		_ptest.YAWORINTE=OutYaw(1);
		_ptest.InitialXYYaw(initx,inity,setyaw);
		_ptest.set_magoffset(BJoffset);//北京磁偏北偏东7°，南特大约为北偏西0.39
	    break;	    
	  }
	  //ahrsqsmf realyaw
	   case 9:{
	    	_ptest.choose_ahrs(3);
		_ptest.YAWORINTE=OutYaw(2);
		_ptest.InitialXY(initx,inity);
		_ptest.set_magoffset(BJoffset);//北京磁偏北偏东7°，南特大约为北偏西0.39
	    break;	    
	  }
	  
	  //ahrsqsmfb yaw
	  case 10:{
	    	_ptest.choose_ahrs(4);
		_ptest.YAWORINTE=OutYaw(0);
		_ptest.InitialXY(initx,inity);
		_ptest.set_magoffset(BJoffset);//北京磁偏北偏东7°，南特大约为北偏西0.39
	    break;	    
	  }
	  //ahrsqsmfb inte
	  case 11:{
	    	_ptest.choose_ahrs(4);
		_ptest.YAWORINTE=OutYaw(1);
		_ptest.InitialXYYaw(initx,inity,setyaw);
		_ptest.set_magoffset(BJoffset);//北京磁偏北偏东7°，南特大约为北偏西0.39
	    break;	    
	  }
	  //ahrsqsmfb real
	   case 12:{
	    	_ptest.choose_ahrs(4);
		_ptest.YAWORINTE=OutYaw(2);
		_ptest.InitialXY(initx,inity);
// 		_ptest.pdrEkf=false;
		_ptest.set_magoffset(BJoffset);//北京磁偏北偏东7°，南特大约为北偏西0.39
	    break;	    
	  }
	  //旋转积分
	   case 13:{
	    	_ptest.choose_ahrs(5);
		_ptest.YAWORINTE=OutYaw(0);
		_ptest.InitialXYYaw(initx,inity,setyaw);//北京磁偏北偏东7°，南特大约为北偏西0.39	     
	  }
	}
	
	if(bool( SHS::Config::get<int> ( "setIos" )))
	{
	  _ptest.set_IOS();
	  cout<<"ios"<<endl;
	}
	_ptest.setCallBack(OnPdrStepCallbackEvent);
// 	cout<<_ptest.cur_index<<endl;
	
	//ifstream inFile("./acc.csv");//index,ax,ay,az,gx,gy,gz,mx,my,mz,grox,groy,groz,pressure,time
// 	ifstream inFile("../data/logfile_2018_11_28_16_21_23.csv");//这个目录指的是运行目录，锁定在当前运行目录下
	ifstream inFile(SHS::Config::get<string> ( "file_name" ));
// 	vector<string> gf;
// 	gf=getFiles("../data/");
// 	ifstream inFile("./test.csv");
	ofstream outFile;
	outFile.open("../data/data.csv");
	string lineStr;
// 	_ptest.InitialXY(10,10);
// 	_ptest.InitialXYYaw(10,10,3.14);

	
// 	_ptest.set_magoffset(-8/180.0*3.14159268);
	int count=0;

	while (getline(inFile, lineStr))  {
	stringstream ss(lineStr);
	string str;  
	vector<double> lineArray;  
	// 按照逗号分隔  
	
	while (getline(ss, str, ','))  
	{
		lineArray.push_back(atof(str.c_str())); 
	}

	count++;
// 	_ptest.adddata( lineArray[10],lineArray[11],lineArray[12], lineArray[1]+lineArray[4],lineArray[2]+lineArray[5],lineArray[3]+lineArray[6],lineArray[7],lineArray[8],lineArray[9],lineArray[14]);
// 	_ptest.adddata( lineArray[10],lineArray[11],lineArray[12], lineArray[1],lineArray[2],lineArray[3],lineArray[4],lineArray[5],lineArray[6],lineArray[7],lineArray[8],lineArray[9],lineArray[14]);	//吕总		
// 	_ptest.adddata( lineArray[7],lineArray[8],lineArray[9], lineArray[1],lineArray[2],lineArray[3],lineArray[4],lineArray[5],lineArray[6],lineArray[10],lineArray[11],lineArray[12],lineArray[27]);	//采集DAT		
// 	_ptest.adddata(lineArray[0],lineArray[1],lineArray[2],lineArray[3],lineArray[4],lineArray[5],lineArray[6],lineArray[7],lineArray[8],lineArray[9],lineArray[10],lineArray[11],lineArray[12]);
	if(bool( SHS::Config::get<int> ( "setIos" )))
	{
	_ptest.adddata( lineArray[7],lineArray[8],lineArray[9], lineArray[1],lineArray[2],lineArray[3],lineArray[4],lineArray[5],lineArray[6],lineArray[10],lineArray[11],lineArray[12],lineArray[25]);	//采集DAT		
	
	}
	else{
	  if(!bool( SHS::Config::get<int> ( "DAT" )))
	  {
	_ptest.adddata( lineArray[10],lineArray[11],lineArray[12], lineArray[1],lineArray[2],lineArray[3],lineArray[4],lineArray[5],lineArray[6],lineArray[7],lineArray[8],lineArray[9],lineArray[13]);	//getsensor数据			    
	  }
	  else{
	_ptest.adddata( lineArray[7],lineArray[8],lineArray[9], lineArray[1],lineArray[2],lineArray[3],lineArray[4],lineArray[5],lineArray[6],lineArray[10],lineArray[11],lineArray[12],lineArray[27]);	//DAT安卓的数据		    
	  }
	}
// 	

// 	test=angle2quat(_ptest.atest->Att(2),_ptest.atest->Att(0),_ptest.atest->Att(1));
// 	test=test.inverse();
	test=Eigen::Quaterniond(_ptest.atest->quaternion.w(),-_ptest.atest->quaternion.y(),-_ptest.atest->quaternion.x(),_ptest.atest->quaternion.z());
	//以这种方式是可以对上的，使得安卓正常对上了
// 	test=_ptest.atest->quaternion;
// 	test.x()*=-1;
// 	test.y()*=-1;
// 	test=test.conjugate();
// 	cout<<"------"<<endl;
// 	cout<<_ptest.atest->Att*180.0/M_PI<<endl;
	dst.SetStringContent("Realyaw:"+to_string(_ptest.atest->GetRealYaw()*180.0/M_PI)+"\n"+"yaw:"+to_string(_ptest.atest->Att(2)*180.0/M_PI)+"\n"+"pitch:"+to_string(_ptest.atest->Att(1)*180.0/M_PI)+"\n"+"roll:"+to_string(_ptest.atest->Att(0)*180.0/M_PI));
// 	getchar();
 	cv::Affine3d PhoneAtt(
            cv::Affine3d::Mat3( test.toRotationMatrix()(0,0),test.toRotationMatrix()(1,0),test.toRotationMatrix()(2,0),
				test.toRotationMatrix()(0,1),test.toRotationMatrix()(1,1),test.toRotationMatrix()(2,1),
				test.toRotationMatrix()(0,2),test.toRotationMatrix()(1,2),test.toRotationMatrix()(2,2)
            ), 
            cv::Affine3d::Vec3(
                0,0,0
            )
        );
	
	cv::Affine3d World(
            cv::Affine3d::Mat3( worldq.toRotationMatrix()(0,0),worldq.toRotationMatrix()(1,0),worldq.toRotationMatrix()(2,0),
				worldq.toRotationMatrix()(0,1),worldq.toRotationMatrix()(1,1),worldq.toRotationMatrix()(2,1),
				worldq.toRotationMatrix()(0,2),worldq.toRotationMatrix()(1,2),worldq.toRotationMatrix()(2,2)
            ), 
            cv::Affine3d::Vec3(
                0,0,0
            )
        );   
	dst.SetCurrentFrame(PhoneAtt,World);
// 	_ptest.atest->Att(2)*180.0/M_PI
// 	dst.SetStringContent("ss");
	if (_ptest.ISSTEP)
	{
// 		cout<<lineArray[13]<<endl;
// 		_ptest.setXY(20,20);
// 		cout<<count<<"step"<<_ptest.get_X()<<","<<_ptest.get_Y()<<","<<_ptest.get_SL()<<","<<_ptest.get_YAW()<<endl;		
//  		outFile<<_ptest.get_X()<<","<<_ptest.get_Y()<<","<<_ptest.get_SL()<<","<<_ptest.get_YAW()<<","<<_ptest.get_deta_angle()<<","<<_ptest.cur_index<<","<<_ptest.cur_time<<endl;
// 		cout<<_ptest.get_X()<<","<<_ptest.get_Y()<<","<<_ptest.get_SL()<<","<<_ptest.get_YAW()<<","<<_ptest.get_deta_angle()<<","<<_ptest.cur_index<<","<<_ptest.cur_time<<endl;
	}
	
	
	}
	stop = time(NULL);
	cout<<stop-start<<"s time"<<endl;
// 	for (unsigned int i = 0; i < pdresult.size(); i++)
// 	{
// 		outFile<<pdresult[i].x<<","<<pdresult[i].y<<","<<pdresult[i].sl<<","<<pdresult[i].yaw<<endl;
// 		cout<<i<<endl;
// 	}
	cout<<"-------------"<<mode<<endl;
	inFile.close();
	outFile.close();
// 	getchar();
	cout<<"-------------"<<mode<<endl;
	return 0;

}

int main2(int argc, char** argv)
{
  deque<float> ndew={1.2f, 2.5f, 5.6f, -2.5f};
  double mean1 = 0., variance1 = 0., stddev1 = 0.;  
  meanStdDev(ndew, &mean1, &variance1, &stddev1);
	cout<<mean1<<endl;
	cout<<variance1<<endl;
	cout<<stddev1<<endl;
	cout<<"--------"<<endl;
// 	cout<<argv[1]<<endl;
//   	getchar();
	return 0;
}
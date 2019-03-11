#include <fstream>
#include "common_include.h"
#include "attitudeobserver.h"
#include "pdrsim.h"
#include <fstream>

void OnFloorStepCallbackEvent(int floorType, int upOrDown, double k, double a ,int current_floorIndex)
{
     cout<<floorType<<","<<upOrDown<<","<<","<<k<<","<<a<<","<<current_floorIndex<<endl;
}

int main1(int argc, char** argv){
  SHS::AttitudeObserver tatt(11);
  ifstream inFile("/home/aaron/projects/SHS/data/tos/楼梯指纹/F11-F1/301F11-1aWCJ1_sensor_raw_5B1A5767.csv");
  ofstream outFile;
  outFile.open("../data/testp.csv");
  tatt.addFloorHeight(4.3);
  tatt.addFloorHeight(7.8);
  for(int i=0;i<10;i++)
  {
    tatt.addFloorHeight(4.3);
  }
  tatt.updateFloorHeightMatrix();

  int tco=0;
  	string lineStr;
  	while (getline(inFile, lineStr)) 
	{
	  stringstream ss(lineStr);
	  string str;  
	  vector<double> lineArray;  
	  // 按照逗号分隔  
	  
	  while (getline(ss, str, ','))  
	  {
	    lineArray.push_back(atof(str.c_str())); 
	  }
	
	  double acc_norm=sqrt(lineArray[1]*lineArray[1]+lineArray[2]*lineArray[2]+lineArray[3]*lineArray[3]);
	  tatt.addData(acc_norm,lineArray[24],lineArray[14],lineArray[27]);
	  if(tatt.pressure_.size()>0){
	    outFile<<tatt.pressure_.back().time_r-tatt.pressure_[0].time_r<<","<<tatt.pressure_.back().value_flitered<<endl;	  
	    
	  }    
	 }
  
  	inFile.close();
	outFile.close();
// 	getchar();
	return 0;
}

int main2(int argc, char** argv){
  SHS::PDRSIM tatt;
  tatt.InitFloorModule(11);
  ifstream inFile("/home/aaron/projects/SHS/data/tos/楼梯指纹/F11-F1/301F11-1aWCJ1_sensor_raw_5B1A5767.csv");
  ofstream outFile;
  outFile.open("../data/testp.csv");
  tatt.floorModuleAddFloorHeight(4.3);
  tatt.floorModuleAddFloorHeight(7.8);
  for(int i=0;i<10;i++)
  {
    tatt.floorModuleAddFloorHeight(4.3);
  }
  tatt.floorModuleUpdateFloorHeightMatrix();
  tatt.setFloorCallBack(OnFloorStepCallbackEvent);
  int tco=0;
  	string lineStr;
  	while (getline(inFile, lineStr)) 
	{
	  stringstream ss(lineStr);
	  string str;  
	  vector<double> lineArray;  
	  // 按照逗号分隔  
	  
	  while (getline(ss, str, ','))  
	  {
	    lineArray.push_back(atof(str.c_str())); 
	  }
	
	  double acc_norm=sqrt(lineArray[1]*lineArray[1]+lineArray[2]*lineArray[2]+lineArray[3]*lineArray[3]);
	  tatt.floorModuleAddData(acc_norm,lineArray[24],lineArray[14],lineArray[27]);
		/*	  if(tatt.pressure_.size()>0){
	    outFile<<tatt.pressure_.back().time_r-tatt.pressure_[0].time_r<<","<<tatt.pressure_.back().value_flitered<<endl;	  
	    
	  } */   
	  
	}
  
  inFile.close();
	outFile.close();
	// 	getchar();
	return 0;
}

int main(int argc, char **argv)
{
	cout<<"hellovscode333"<<endl;
}
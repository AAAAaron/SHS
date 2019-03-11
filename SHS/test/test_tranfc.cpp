#include "ktransfcoordinate.h"
#include "iostream"

int main(int argc, char** argv)
{
  	double xxds_ori_llh[3]= {39.895667,116.362129,0};
	double to_llh[3] ={39.895667,116.362772,0};
	double center_nt[3]={47.22426029,-1.63162863,0};
	KTransfCoordinate ktemp;
	double distance =ktemp.cal_distance_2llh(xxds_ori_llh,to_llh);
	printf("distance=%.10f \n",distance);
	
	
	double toresult[3]={0,0,0};
	ktemp.llh2enu(to_llh,xxds_ori_llh,toresult);

	printf("to_llh的llh2enu结果是 1=%.10f,2=%.10f,3=%.10f \n",toresult[0],toresult[1],toresult[2]);
	
	double llh2result[3]={0,0,0};
	ktemp.enu2llh(toresult,xxds_ori_llh,llh2result);

	printf("to_llh的llh2enu的enu2llh结果是 1=%.10f,2=%.10f,3=%.10f \n",llh2result[0],llh2result[1],llh2result[2]);
	
	double llh2result2[3]={0,0,0};
	llh2result2[0]=55;
	ktemp.posupdate_LLH(xxds_ori_llh,llh2result2);
	printf("posupdate_LLH的结果是 1=%.10f,2=%.10f,3=%.10f \n",xxds_ori_llh[0],xxds_ori_llh[1],xxds_ori_llh[2]);
	
	
	double xxds_ori_llh2[3]= {39.895667,116.362129,0};
	printf("cal_distance_2llh=%.10f \n",ktemp.cal_distance_2llh(xxds_ori_llh2,to_llh));
	double xxds_ori_llh3[3]= {39.895667,116.362129,0};
	double enusss[3]= {55.0,0,0};
	double toresult22[3]={0.0,0,0};
	ktemp.enu2llh(enusss,xxds_ori_llh3,toresult22);

	printf("enusss的enu2llh结果是 1=%.10f,2=%.10f,3=%.10f \n",toresult22[0],toresult22[1],toresult22[2]);
	
  return 0;
}
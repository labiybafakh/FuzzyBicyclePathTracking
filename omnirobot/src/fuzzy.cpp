#include "ros/ros.h"
#include "math.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include "stdio.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"


#define RADS	57.2957795f


#define R 20

using namespace std;

float V1,V2,V3;
double tsin,tcos;
double TRad;
double TempX,TempY;
double PosX,PosY,Theta;
float ErX,ErY,ErT,ErD,DErT,ErT_1,PID,PIDX,PIDY,PIDT,Xn,Yn,Tn,IErRes,ErDX,ErDY,LErY,LErX,LErT;
float Gyro,ControlX,ControlY,ControlT,XX,YY,Dx,Dy;
float LastX,LastY;
float TopX,TopY,TopT,TT,ErDMax,ResTarget,ResRobot,ErRes,DErRes,ErRes_1;
float sudut_Theta1,sudut_Theta2,sudut_Theta3;
float deltaX,deltaY;
float constanta,cc;
float A,B,C,b2,det,akarx,ya,yb,xa,xb;
float M1,M2,Mgaris;
float xp,yp,varX;
float valspeed,cosD,sinD,selisihX,selisihY,arah_gerak1,arah_gerak2,arah;
float jarak_robot,jarak_target,jarak_tempuh;
float flagrun=0;
float flags=0;
float rest=0;
float runqueue=0;
float OutMF1[5],OutMF2[5];
float pembilang,penyebut;
float defuzz;
float inputMF1,inputMF2;
float sudutawal,flagawal=0;
float rmsX,rmsY;
float Rfuzzy,Vfuzzy;
float target_distance=0,erorjalur=0;

float Small[10],Medium[10],Big[10],SuperBig[10];
float VerySlow[10],Slow[10],SMedium[10],Fast[10],VeryFast[10];
float Rule[17];
int i=0,k=0;
float maxK=0,maxS=0,maxB=0,maxSB=0;
float maxVS,maxSl,maxM,maxF,maxVF;
float RFuzz1,RFuzz2,RFuzz3,RFuzz4,ROutFuzz[1000];
float VFuzz1,VFuzz2,VFuzz3,VFuzz4,VFuzz5,VOutFuzz[1000];
float RPembilangCoa,RPenyebutCoa,RCOA;
float VPembilangCoa,VPenyebutCoa,VCOA;

void FuzzDefuzz(float nilai1,float nilai2);

void DistanceCallback(const std_msgs::Int32::ConstPtr& dataR){
	target_distance = dataR->data;
}

void PathCallback(const std_msgs::Int32::ConstPtr& dataV){
	erorjalur = dataV->data;

    FuzzDefuzz(target_distance,erorjalur);

}


void MembershipFunction(float nilai,float nilai2){
	

	if(nilai<20) OutMF1[0]= 1;
	else if(nilai>=20 && nilai<=30) OutMF1[0]= ((30-nilai)/10);
	else OutMF1[0]=0;

	if(nilai<=20 || nilai>=50) OutMF1[1]=0;
	else if(nilai>20 && nilai<=35) OutMF1[1]=((nilai-20)/(35-20));
	else if(nilai>35 && nilai<50) OutMF1[1]=((50-nilai)/(50-35));

	if(nilai<=40 || nilai>=70) OutMF1[2]=0;
	else if(nilai>40 && nilai<=55) OutMF1[2]=((nilai-40)/(55-40));
	else if(nilai>55 && nilai<70) OutMF1[2]=((70-nilai)/(70-55));

	if(nilai>=70) OutMF1[3]=1;
	else if(nilai<=70 && nilai>=60) OutMF1[3]=((nilai-60)/(10));
	else OutMF1[3]=0; 

	if(nilai2<20) OutMF2[0]= 1;
	else if(nilai2>=20 && nilai2<=30) OutMF2[0]= ((30-nilai2)/10);
	else OutMF2[0]=0;

	if(nilai2<=20 || nilai2>=50) OutMF2[1]=0;
	else if(nilai2>20 && nilai2<=35) OutMF2[1]=((nilai2-20)/(35-20));
	else if(nilai2>35 && nilai2<50) OutMF2[1]=((50-nilai2)/(50-35));

	if(nilai2<=40 || nilai2>=70) OutMF2[2]=0;
	else if(nilai2>40 && nilai2<=55) OutMF2[2]=((nilai2-40)/(55-40));
	else if(nilai2>55 && nilai2<70) OutMF2[2]=((70-nilai2)/(70-55));

	if(nilai2>=70) OutMF2[3]=1;
	else if(nilai2<=70 && nilai2>=60) OutMF2[3]=((nilai2-60)/(10));
	else OutMF2[3]=0; 
}

float MFOut(float nilai,int index){
	
	float value;
	
	if(index==1){
		if(nilai<10) value= 1;
		else if(nilai>=10 && nilai<=15) value= ((15-nilai)/5);
		else value=0;
	}
	
	else if(index==2){		
		if(nilai<=12.5 || nilai>=27.5) value=0;
		else if(nilai>12.5 && nilai<=20) value=((nilai-12.5)/(20-12.5));
		else if(nilai>20 && nilai<27.5) value=((27.5-nilai)/(27.5-20));
	}
	else if(index==3){		
		if(nilai<=25 || nilai>=40) value=0;
		else if(nilai>25 && nilai<=32.5) value=((nilai-25)/(32.5-25));
		else if(nilai>32.5 && nilai<40) value=((40-nilai)/(40-32.5));
	}
	else if(index==4){
		if(nilai>40) value=1;
		else if(nilai<=40 && nilai>=35) value=((nilai-35)/(5));
		else value=0;
	}
	
	return value; 
}

float MFOut2(float nilai,int index){
	
	float value;
	
	if(index==1){
		if(nilai<50) value=1;
		else if(nilai>=50 && nilai<=70) value= ((70-nilai)/20);
		else value=0;
	}
	
	else if(index==2){		
		if(nilai<=60 || nilai>=120) value=0;
		else if(nilai>60 && nilai<=90) value=((nilai-60)/(30));
		else if(nilai>90 && nilai<120) value=((120-nilai)/(30));
	}
	else if(index==3){		
		if(nilai<=110 || nilai>=170) value=0;
		else if(nilai>110 && nilai<=140) value=((nilai-110)/(30));
		else if(nilai>140 && nilai<170) value=((170-nilai)/(30));
	}
	else if(index==4){		
		if(nilai<=170 || nilai>=750) value=0;
		else if(nilai>170 && nilai<=200) value=((nilai-170)/(30));
		else if(nilai>200 && nilai<230) value=((230-nilai)/(30));
	}
	else if(index==5){
		if(nilai>290) value=1;
		else if(nilai<=240 && nilai>=220) value=((nilai-220)/(20));
		else value=0;
	}
	
	return value; 
}



/*
float MFOut(float nilai,int index){
	
	float value;
	
	if(index==1){
		if(nilai<20) value= 1;
		else if(nilai>=20 && nilai<=30) value= ((30-nilai)/10);
		else value=0;
	}
	
	else if(index==2){		
		if(nilai<=20 || nilai>=50) value=0;
		else if(nilai>20 && nilai<=35) value=((nilai-20)/(35-20));
		else if(nilai>35 && nilai<50) value=((50-nilai)/(50-35));
	}
	else if(index==3){		
		if(nilai<=40 || nilai>=70) value=0;
		else if(nilai>40 && nilai<=55) value=((nilai-40)/(55-40));
		else if(nilai>55 && nilai<70) value=((70-nilai)/(70-55));
	}
	else if(index==4){
		if(nilai>=70) value=1;
		else if(nilai<=70 && nilai>=60) value=((nilai-60)/(10));
		else value=0;
	}
	
	return value; 
}
*/


void FuzzDefuzz(float nilai1,float nilai2){

	
	MembershipFunction(nilai1,nilai2);
	
	Rule[0]=min(OutMF1[0],OutMF2[0]);
	Rule[1]=min(OutMF1[0],OutMF2[1]);
	Rule[2]=min(OutMF1[0],OutMF2[2]);	
	Rule[3]=min(OutMF1[0],OutMF2[3]);
	Rule[4]=min(OutMF1[1],OutMF2[0]);
	Rule[5]=min(OutMF1[1],OutMF2[1]);
	Rule[6]=min(OutMF1[1],OutMF2[2]);	
	Rule[7]=min(OutMF1[1],OutMF2[3]);
	Rule[8]=min(OutMF1[2],OutMF2[0]);
	Rule[9]=min(OutMF1[2],OutMF2[1]);
	Rule[10]=min(OutMF1[2],OutMF2[2]);	
	Rule[11]=min(OutMF1[2],OutMF2[3]);
	Rule[12]=min(OutMF1[3],OutMF2[0]);
	Rule[13]=min(OutMF1[3],OutMF2[1]);
	Rule[14]=min(OutMF1[3],OutMF2[2]);	
	Rule[15]=min(OutMF1[3],OutMF2[3]);

	SMedium[0]=Rule[0];Slow[0]=Rule[2];Slow[1]=Rule[2];VerySlow[0]=Rule[3];
	Fast[0]=Rule[4];SMedium[1]=Rule[5];Slow[2]=Rule[6];Slow[3]=Rule[7];
	VeryFast[0]=Rule[8];Fast[1]=Rule[9];SMedium[2]=Rule[10];Slow[4]=Rule[11];
	VeryFast[1]=Rule[12];Fast[2]=Rule[13];SMedium[3]=Rule[14];Slow[5]=Rule[15];

	
	Small[0]=Rule[0];Small[1]=Rule[2];Small[2]=Rule[2];Small[3]=Rule[3];
	Medium[0]=Rule[4];Medium[1]=Rule[5];Small[4]=Rule[6];Small[5]=Rule[7];
	Big[0]=Rule[8];Big[1]=Rule[9];Medium[2]=Rule[10];Small[6]=Rule[11];
	SuperBig[0]=Rule[12];SuperBig[1]=Rule[13];Big[3]=Rule[14];Medium[3]=Rule[15];

	maxVS=VerySlow[0];
	maxSl=max(Slow[0],max(Slow[1],max(Slow[2],max(Slow[3],max(Slow[4],Slow[5])))));
	maxM=max(SMedium[0],max(SMedium[1],max(SMedium[2],SMedium[3])));
	maxF=max(Fast[0],max(Fast[1],Fast[2]));
	maxVF=max(VeryFast[0],VeryFast[1]);
	

	maxK = max(Small[0],max(Small[1],max(Small[2],max(Small[3],max(Small[4],max(Small[5],Small[6]))))));
	maxS = max(Medium[0],max(Medium[1],max(Medium[2],Medium[3])));
	maxB = max(Big[0],max(Big[1],max(Big[2],Big[3])));
	maxSB= max(SuperBig[0],SuperBig[1]);

	/*
	Small[0]=Rule[2];
	Small[1]=Rule[3];
	Small[2]=Rule[7];

	

	Medium[0]=Rule[1];
	Medium[1]=Rule[6];
	Medium[2]=Rule[11];
	Medium[3]=Rule[15];

	

	Big[0]=Rule[0];
	Big[1]=Rule[5];
	Big[2]=Rule[9];
	Big[3]=Rule[10];
	Big[4]=Rule[13];
	Big[5]=Rule[14];

	SuperBig[0]=Rule[4];
	SuperBig[1]=Rule[8];
	SuperBig[2]=Rule[12];

	maxK = max(Small[0],max(Small[1],Small[2]));
	maxS = max(Medium[0],max(Medium[1],max(Medium[2],Medium[3])));
	maxB = max(Big[0],max(Big[1],max(Big[2],max(Big[3],max(Big[4],Big[5])))));
	maxSB= max(SuperBig[0],max(SuperBig[1],SuperBig[2]));	

	*/	


	/*
	Small[0]=Rule[2];
	Small[1]=Rule[3];
	Small[2]=Rule[7];
	
	Medium[0]=Rule[1];
	Medium[1]=Rule[6];
	Medium[2]=Rule[11];
	Medium[3]=Rule[15];
	
	Big[0]=Rule[0];
	Big[1]=Rule[5];
	Big[2]=Rule[9];
	Big[3]=Rule[10];
	Big[4]=Rule[13];
	Big[5]=Rule[14];
	
	SuperBig[0]=Rule[4];
	SuperBig[1]=Rule[8];
	SuperBig[2]=Rule[12];

	maxK = max(Small[0],max(Small[1],Small[2]));
	maxS = max(Medium[0],max(Medium[1],max(Medium[2],Medium[3])));
	maxB = max(Big[0],max(Big[1],max(Big[2],max(Big[3],max(Big[4],Big[5])))));
	maxSB= max(SuperBig[0],max(SuperBig[1],SuperBig[2]));	
	*/	

	for(k=0;k<=400;k++){
		
			if(MFOut2(k,1)>maxVS) 	VFuzz1=maxVS;
			else					VFuzz1=MFOut2(k,1);

			if(MFOut2(k,2)>maxSl) 	VFuzz2=maxSl;
			else					VFuzz2=MFOut2(k,2);

			if(MFOut2(k,3)>maxM) 	VFuzz3=maxM;
			else					VFuzz3=MFOut2(k,3);

			if(MFOut2(k,4)>maxF) 	VFuzz4=maxF;
			else					VFuzz4=MFOut2(k,4);
			
			if(MFOut2(k,5)>maxVF) 	VFuzz5=maxVF;
			else					VFuzz5=MFOut2(k,5);
			

			VOutFuzz[k]=max(VFuzz1,max(VFuzz2,max(VFuzz3,max(VFuzz4,VFuzz5))));

			VPembilangCoa += (VOutFuzz[k]*k);
			VPenyebutCoa += VOutFuzz[k];
			VCOA = VPembilangCoa/VPenyebutCoa;
	}

	    VPembilangCoa=0;VPenyebutCoa=0;
		Vfuzzy=VCOA;
		k=0;


	for(i=0;i<=50;i++){
		
			if(MFOut(i,1)>maxK) 	RFuzz1=maxK;
			else					RFuzz1=MFOut(i,1);

			if(MFOut(i,2)>maxS) 	RFuzz2=maxS;
			else					RFuzz2=MFOut(i,2);

			if(MFOut(i,3)>maxB) 	RFuzz3=maxB;
			else					RFuzz3=MFOut(i,3);

			if(MFOut(i,4)>maxSB) 	RFuzz4=maxSB;
			else					RFuzz4=MFOut(i,4);
			
			ROutFuzz[i]=max(RFuzz1,max(RFuzz2,max(RFuzz3,RFuzz4)));

			RPembilangCoa += (ROutFuzz[i]*i);
			RPenyebutCoa += ROutFuzz[i];
			RCOA = RPembilangCoa/RPenyebutCoa;
	}
		RPembilangCoa=0;RPenyebutCoa=0;
		Rfuzzy=RCOA;
		i=0;


	//Defuzzifikasi();
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "Fuzzy");
  ros::NodeHandle n;
  ros::NodeHandle vel;
  ros::NodeHandle fuzzy;

  ros::Subscriber sub_distance = fuzzy.subscribe("ErDistance",1000,DistanceCallback);
  ros::Subscriber sub_path = fuzzy.subscribe("ErPath",1000,PathCallback);
  ros::Publisher vfuzzy_pub = fuzzy.advertise<std_msgs::Int32>("Vfuzzy",1000);
  ros::Publisher rfuzzy_pub = fuzzy.advertise<std_msgs::Int32>("Rfuzzy",1000);

  geometry_msgs::Twist msg;
  std_msgs::Int32 data1;
  std_msgs::Int32 data2;

	
  sudutawal=Theta;
  flags=0;

  ros::Rate r(100);
  int s;
  while(ros::ok){
    
	
    data1.data = Vfuzzy;
    data2.data = Rfuzzy;
    

    vfuzzy_pub.publish(data1);
    rfuzzy_pub.publish(data2);

    printf("\n%f\t%f\t%f\t%f",target_distance,erorjalur,Vfuzzy,Rfuzzy);


    ros::spinOnce();
	r.sleep();
  }

  return 0;
}

#include "ros/ros.h"
#include "math.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Int8.h"
#include "stdio.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"


#define RADS	57.2957795f


#define R 110

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
float Rfuzzy=200,Vfuzzy=200;
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

void imuCallback(const std_msgs::Int8::ConstPtr& dataimu){
	Theta = dataimu->data;
	TRad  = Theta / RADS;
	tcos  = cos(TRad);
	tsin  = sin(TRad);
}

void chatterCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
  TempX = (msg->transform.translation.x);
  TempY = (msg->transform.translation.y);
  

  PosX  += (TempX * tcos) + (TempY * tsin);
  PosY  += (TempY * tcos) - (TempX * tsin);

  printf("\n%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f",ErX,ErY,selisihX,selisihY,PosX,PosY,jarak_tempuh,runqueue);
	// if(runqueue==10)printf("\nPATH SELESAI DIJALANKAN");
	// else
	// printf("\nPath:%d\tSpeed:%.2f rad/s \tJari-Jari:%.2f cm\t\tPosX:%.2f cm\tPosY:%.2f cm\tTheta:%.2f derajat",(int)runqueue,Vfuzzy,Rfuzzy,xp,yp,Theta);

/*
	
  ROS_INFO("Seq: [%d]", msg->header.seq);
  ROS_INFO("Position-> PosX: [%f], PosY: [%f], Theta: [%f]", PosX,PosY,Theta);
  ROS_INFO("Speed-> ControlX: [%f], ControlY: [%f], ControlT: [%f]", ControlX,ControlY,ControlT);
  ROS_INFO("Jarak-> ErT: [%f], sudut_Theta1: [%f], sudut_Theta2: [f]",jarak_tempuh,sudut_Theta1,sudut_Theta2);
*/
}

void InversKinematic(int kecX , int kecY ,int kecT){
	V1=((-0.3333*kecX) - (0.5774*kecY) - (1.001*kecT));
	V2=((-0.3333*kecX) + (0.5774*kecY) - (1.001*kecT)); 
  	V3=((0.6667*kecX) - 0 - (1.001*kecT));		

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

	if(k<=400){
		
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

	else{
		VPembilangCoa=0;VPenyebutCoa=0;
		Vfuzzy=VCOA;
		k=0;

	}

	if(i<=50){
		
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
	else{
		RPembilangCoa=0;RPenyebutCoa=0;
		Rfuzzy=RCOA;
		i=0;

	}

	//Defuzzifikasi();
}



void FuzzyBP(float SPX, float SPY, float SPTeta, float respon){	

	if(flags==0){
		deltaX = SPX - PosX;
		deltaY = SPY - PosY;
		ErX = SPX - PosX;
		ErY = SPY - PosY;
	
		Dx = SPX - LastX;
		Dy = SPY - LastY;
	
		if(Dx!=0 && Dy!=0){
			M1=Dy/Dx;
			M2=-1/M1;
			Mgaris=Dy/Dx;
		}
		else {M1=0; M2=0; Mgaris=0;}
		

		jarak_robot = sqrt((PosX*PosX)+(PosY*PosY));
		jarak_target = sqrt((SPX*SPX)+(SPY*SPY));
		jarak_tempuh = sqrt ((ErX*ErX)+(ErY*ErY));
		ResTarget = ((atan2((SPX-LastX),(SPY-LastY)))*RADS);
		target_distance = sqrt (((SPX-PosX)*(SPX-PosX))+((SPY-PosY)*(SPY-PosY)));

		ErT = (SPTeta - Theta)*-1;
		flags=1;
	}

	if(jarak_tempuh > 2){
		
		flagrun=0;

		constanta=SPY-(Mgaris*SPX);
		cc=constanta-PosY;
		
		if(Dx==0){
			xp=LastX;
			yp=PosY;
		}
		else if(Dy==0){
			xp=PosX;
			yp=LastY;
		}
		else{
			xp=((M1*SPX)-(M2*PosX)+PosY-SPY)/(M1-M2);
			yp=(M1*(xp-SPX))+SPY;
		}
	
		if(Dx!=0){
			A=1+(Mgaris*Mgaris);
			B=(-2*PosX)+(2*Mgaris*constanta)-(2*Mgaris*PosY);
			C=(PosX*PosX)+(cc*cc)-(Rfuzzy*Rfuzzy);
	
			b2=B*B;
			det=b2-(4*A*C);
			akarx=sqrt(fabs(det));
			xa=(-B+akarx)/(2*A);
			xb=(-B-akarx)/(2*A);

			ya=Mgaris*xa+constanta;
			yb=Mgaris*xb+constanta;
		}
		else {
			varX=SPX-PosX;
			A=1;
			B=-2*PosY;
			C=(PosY*PosY)-(Rfuzzy*Rfuzzy)+(varX*varX);

			b2=B*B;
			det=b2-(4*A*C);
			akarx=sqrt(fabs(det));
			ya=(-B+akarx)/(2*A);
			yb=(-B-akarx)/(2*A);

			xa=SPX; xb=SPX;
		}
		sudut_Theta1=(atan2((xa-PosX),(ya-PosY)))*RADS;
		sudut_Theta2=(atan2((xb-PosX),(yb-PosY)))*RADS;
		
		arah_gerak1=(atan2((xa-xp),(ya-yp)))*RADS;
		arah_gerak2=(atan2((xb-xp),(yb-yp)))*RADS;
		
		if((int)arah_gerak1==(int)ResTarget){arah=1;}
		else if((int)arah_gerak2==(int)ResTarget){arah=0;}
		
		if(arah==1){

			//cosD=cos((sudut_Theta1)/RADS);
			//sinD=sin((sudut_Theta1)/RADS);
			cosD=cos((sudut_Theta1-Theta)/RADS);
			sinD=sin((sudut_Theta1-Theta)/RADS);			
			selisihX=SPX-xa; selisihY=SPY-ya;

		}
		else if(arah==0){
			cosD=cos((sudut_Theta2-Theta)/RADS);
			sinD=sin((sudut_Theta2-Theta)/RADS);

			selisihX=SPX-xb; selisihY=SPY-yb;

		}
		
		//jarak_tempuh = sqrt ((selisihX*selisihX)+(selisihY*selisihY));
		valspeed = Vfuzzy; 


	  		ControlY = valspeed*cosD;		  	
	  		ControlX = valspeed*sinD;
  
		ErT = (SPTeta - Theta);
		DErT= ErT - LErT;
		LErT=ErT;
		ControlT = ((ErT*respon) + (DErT *2));
	
		rmsX = (PosX-xp)*(PosX-xp);
		rmsY = (PosY-yp)*(PosY-yp);
		erorjalur=sqrt(rmsX+rmsY);

		target_distance = sqrt (((SPX-PosX)*(SPX-PosX))+((SPY-PosY)*(SPY-PosY)));
		if(target_distance>100)target_distance=100;		

		inputMF1 = target_distance;
		inputMF2 = erorjalur;
		
		FuzzDefuzz(inputMF1,inputMF2);
	}

	else{
		flagrun=1;
		LastX=SPX, LastY=SPY;
	}

}

void bicycle_path(float SPX, float SPY, float SPTeta, float kec_max, float jari2, float respon){	

	if(flags==0){
		deltaX = SPX - PosX;
		deltaY = SPY - PosY;
		ErX = SPX - PosX;
		ErY = SPY - PosY;
	
		Dx = SPX - LastX;
		Dy = SPY - LastY;
	
		if(Dx!=0 && Dy!=0){
			M1=Dy/Dx;
			M2=-1/M1;
			Mgaris=Dy/Dx;
		}
		else {M1=0; M2=0; Mgaris=0;}
		

		jarak_robot = sqrt((PosX*PosX)+(PosY*PosY));
		jarak_target = sqrt((SPX*SPX)+(SPY*SPY));
		jarak_tempuh = sqrt ((ErX*ErX)+(ErY*ErY));
		ResTarget = ((atan2((SPX-LastX),(SPY-LastY)))*RADS);
		ErT = (SPTeta - Theta)*-1;
		flags=1;
	}

	if(jarak_tempuh > 2){
		
		flagrun=0;

		constanta=SPY-(Mgaris*SPX);
		cc=constanta-PosY;
		
		if(Dx==0){
			xp=LastX;
			yp=PosY;
		}
		else if(Dy==0){
			xp=PosX;
			yp=LastY;
		}
		else{
			xp=((M1*SPX)-(M2*PosX)+PosY-SPY)/(M1-M2);
			yp=(M1*(xp-SPX))+SPY;
		}
	
		if(Dx!=0){
			A=1+(Mgaris*Mgaris);
			B=(-2*PosX)+(2*Mgaris*constanta)-(2*Mgaris*PosY);
			C=(PosX*PosX)+(cc*cc)-(jari2*jari2);
	
			b2=B*B;
			det=b2-(4*A*C);
			akarx=sqrt(fabs(det));
			xa=(-B+akarx)/(2*A);
			xb=(-B-akarx)/(2*A);

			ya=Mgaris*xa+constanta;
			yb=Mgaris*xb+constanta;
		}
		else {
			varX=SPX-PosX;
			A=1;
			B=-2*PosY;
			C=(PosY*PosY)-(jari2*jari2)+(varX*varX);

			b2=B*B;
			det=b2-(4*A*C);
			akarx=sqrt(fabs(det));
			ya=(-B+akarx)/(2*A);
			yb=(-B-akarx)/(2*A);

			xa=SPX; xb=SPX;
		}
		sudut_Theta1=(atan2((xa-PosX),(ya-PosY)))*RADS;
		sudut_Theta2=(atan2((xb-PosX),(yb-PosY)))*RADS;
		
		arah_gerak1=(atan2((xa-xp),(ya-yp)))*RADS;
		arah_gerak2=(atan2((xb-xp),(yb-yp)))*RADS;
		
		if((int)arah_gerak1==(int)ResTarget){arah=1;}
		else if((int)arah_gerak2==(int)ResTarget){arah=0;}
		
		if(arah==1){

			//cosD=cos((sudut_Theta1)/RADS);
			//sinD=sin((sudut_Theta1)/RADS);
			cosD=cos((sudut_Theta1-Theta)/RADS);
			sinD=sin((sudut_Theta1-Theta)/RADS);			
			selisihX=SPX-xa; selisihY=SPY-ya;

		}
		else if(arah==0){
			cosD=cos((sudut_Theta2-Theta)/RADS);
			sinD=sin((sudut_Theta2-Theta)/RADS);

			selisihX=SPX-xb; selisihY=SPY-yb;

		}
		
		jarak_tempuh = sqrt ((selisihX*selisihX)+(selisihY*selisihY));
		valspeed = kec_max; 

		if(valspeed>kec_max) valspeed=kec_max;
		//if(valspeed<20)valspeed=20;

	  		ControlY = valspeed*cosD;		  	
	  		ControlX = valspeed*sinD;
  
		ErT = (SPTeta - Theta);
		DErT= ErT - LErT;
		LErT=ErT;
		ControlT = ((ErT*respon) + (DErT *2))*kec_max;
	
	}

	else{
		flagrun=1;
		LastX=SPX, LastY=SPY;
	}

}

/*

void bicycle_path2(float SPX, float SPY, float SPTeta, float kec_max, float jari2, float respon){	

	ErX = SPX - PosX;
	ErY = SPY - PosY;
	
	Dx = SPX - LErX;
	Dy = SPY - LErY;
	
	if(Dx!=0 && Dy!=0){
		M1=Dy/Dx;
		M2=-1/M1;
		Mgaris=Dy/Dx;
	}
	else {M1=0; M2=0; Mgaris=0;}

	jarak_robot = sqrt((PosX*PosX)+(PosY*PosY));
	jarak_target = sqrt((SPX*SPX)+(SPY*SPY));
	jarak_tempuh = sqrt((ErX*ErX)+(ErY*ErY));
	ResTarget = ((atan2((SPX-LErX),(SPY-LErY)))*RADS);
	ErT = (SPTeta - Theta);

	if(jarak_tempuh > 5){
		flagrun=0;

		ErX = SPX - PosX;
		ErY = SPY - PosY;
		constanta=SPY-(Mgaris*SPX);
		cc=constanta-PosY;
		
		if(Dx==0){
			xp=LErX;
			yp=PosY;
		}
		else if(Dy==0){
			xp=PosX;
			yp=LErY;
		}
		else{
			xp=((M1*SPX)-(M2*PosX)+PosY-SPY)/(M1-M2);
			yp=(M1*(xp-SPX))+SPY;
		}
	
		if(Dx!=0){
			A=1+(Mgaris*Mgaris);
			B=(-2*PosX)+(2*Mgaris*constanta)-(2*Mgaris*PosY);
			C=(PosX*PosX)+(cc*cc)-(jari2*jari2);
	
			b2=B*B;
			det=b2-(4*A*C);
			akarx=sqrt(fabs(det));
			xa=(-B+akarx)/(2*A);
			xb=(-B-akarx)/(2*A);

			ya=Mgaris*xa+constanta;
			yb=Mgaris*xb+constanta;
		}
		else {
			varX=SPX-PosX;
			A=1;
			B=-2*PosY;
			C=(PosY*PosY)-(jari2*jari2)+(varX*varX);

			b2=B*B;
			det=b2-(4*A*C);
			akarx=sqrt(fabs(det));
			ya=(-B+akarx)/(2*A);
			yb=(-B-akarx)/(2*A);

			xa=SPX; xb=SPX;
		}
		sudut_Theta1=(atan2((xa-PosX),(ya-PosY)))*RADS;
		sudut_Theta2=(atan2((xb-PosX),(yb-PosY)))*RADS;
		sudut_Theta3=(atan2((xp-PosX),(yp-PosY)))*RADS;
		
		arah_gerak1=(atan2((xa-xp),(ya-yp)))*RADS;
		arah_gerak2=(atan2((xb-xp),(yb-yp)))*RADS;
		
		if((int)arah_gerak1==(int)ResTarget){arah=1;}
		else if((int)arah_gerak2==(int)ResTarget){arah=0;}
		if(det>=0){
			if(arah==1){
				cosD=cos((sudut_Theta1-Theta)/RADS);
				sinD=sin((sudut_Theta1-Theta)/RADS);
				selisihX=SPX-xa; selisihY=SPY-ya;

			}
			else if(arah==0){
				cosD=cos((sudut_Theta2-Theta)/RADS);
				sinD=sin((sudut_Theta2-Theta)/RADS);
				selisihX=SPX-xb; selisihY=SPY-yb;

			}
		}
		else if(det<0){
			cosD=cos((sudut_Theta3-Theta)/RADS);
			sinD=sin((sudut_Theta3-Theta)/RADS);
			selisihX=SPX-xa; selisihY=SPY-ya;
		}
		
		jarak_tempuh = sqrt ((ErX*ErX)+(ErY*ErY));
		valspeed = kec_max; 

		if(valspeed>kec_max) valspeed=kec_max;
		//if(valspeed<20)valspeed=20;

	  		ControlY = valspeed*cosD;		  	
	  		ControlX = valspeed*sinD;
  
		ErT = (SPTeta - Theta);
		DErT= ErT - ErT_1;
		LErT=ErT_1;
		ControlT = (ErT*respon) + (DErT *2);
	}
	else{
		LErX=SPX, LErY=SPY;
		flagrun=1;
	}

}
*/

void GoingTo(float XTarget, float YTarget, float TTarget, float Speed){

		TopX=Speed;
		TopY=Speed;
		TopT=Speed;

		if(flags==0){
			  Xn=PosX;
			  Yn=PosY;
			  Tn=Theta;



			ErX 	= XTarget - PosX;
			ErY 	= YTarget - PosY;
	

			ErD 	= sqrt((ErX * ErX) + (ErY * ErY));
			ErDMax 	= ErD;
			ErT	= TTarget - Theta;
			DErT=ErT-ErT_1;	
			ErT_1=ErT;


			ErRes		= ResTarget - ResRobot;	

			DErRes 		= ErRes - ErRes_1;

			ErRes_1 	= ErRes;
			
			rest 		= ((atan2(XTarget-Xn, YTarget-Yn))*RADS);				
			ResRobot		= (atan2(ErX, ErY))*RADS;
			  flags=1;
		}
		





		//if(ErDMax-ErD<ErDMax-10){
	
		if(ErRes>2 && flagrun==0){

			ErX 	= XTarget - PosX;
			ErY 	= YTarget - PosY;
			ErT	= TTarget - Theta;
			DErT=ErT-ErT_1;	
			ErT_1=ErT;

			ErD 	= sqrt((ErX * ErX) + (ErY * ErY));
			ErDMax 	= ErD;
/*			
			TT=((ErDMax - ErD)/ErDMax * (TTarget));		

			ErT=TT - Theta;																// HEADINGLOCK

			DErT=ErT-ErT_1;

			ErT_1=ErT;

*/			
			if(rest>2) 	ErRes = ResTarget - ResRobot;
			else if(rest<2)	ErRes = -ResTarget + ResRobot;	

			DErRes 		= ErRes - ErRes_1;

			ErRes_1 	= ErRes;
			
			ResTarget 		= ((atan2(XTarget-Xn, YTarget-Yn))*RADS);				
			ResRobot		= (atan2(ErX, ErY))*RADS;

			if(ResRobot>0)flagrun=0;
			//PID = (20.0 * ErRes) + (50.0 * DErRes);
								
			PID = (20.0 * ErRes) + (15.0 * DErRes);

			PIDT 		= (0.01 * ErT) + (2.0 * DErT);
			//PIDT 		= (0.005 * ErT) + (0.1 * DErT);


			if(PID>1000)					{PID=1000;}
			else if(PID<-1000)		{PID=-1000;}

			if(PIDT>1000)					{PIDT=1000;}
			else if(PIDT<-1000)		{PIDT=-1000;}			

			ControlY	= (cos((ResTarget -PID) / RADS) * TopY);
			ControlX  	= (sin((ResTarget -PID) / RADS) * TopX);
			//tControlT  	= -PIDT * (TopT);


			if(ControlX>=1000) 				ControlX=1000;
			else if(ControlX<=-1000)			ControlX=-1000;
			if(ControlY>=1000) 				ControlY=1000;
			else if(ControlY<=-1000)			ControlY=-1000;
			if(ControlT>=1000) 				ControlT=1000;
			else if(ControlT<=-1000)			ControlT=-1000;

			if(flagrun==1){ControlX=ControlY=ControlT=0;}


		}
		else{
			flagrun=1;
			//runqueue++;
		}

}

void GoingTo1(float XTarget, float YTarget, float TTarget, float Speed){

		TopX=Speed;
		TopY=Speed;
		TopT=Speed;

		if(flags==0){
		  Xn=PosX;
		  Yn=PosY;
		  Tn=Theta;
		  flags=1;
		}
		


			ErX 	= XTarget - PosX;
			ErY 	= YTarget - PosY;
	

			ErD 	= sqrt((ErX * ErX) + (ErY * ErY));
			ErDMax 	= ErD;
			ErT	= TTarget - Theta;
			DErT=ErT-ErT_1;	
			ErT_1=ErT;
			/*
			TT=((ErDMax - ErD)/ErDMax * (TTarget));		

			ErT=TT - Theta;																// HEADINGLOCK

			DErT=ErT-ErT_1;

			ErT_1=ErT;
*/

			ErRes		= ResTarget - ResRobot;	

			DErRes 		= ErRes - ErRes_1;

			ErRes_1 	= ErRes;
			
			ResTarget 		= ((atan2(XTarget-Xn, YTarget-Yn))*RADS);				
			ResRobot		= (atan2(ErX, ErY))*RADS;


		if(ErDMax-ErD<ErDMax-10){
	
		//if(ResRobot>0 && flagrun==0){

			ErX 	= XTarget - PosX;
			ErY 	= YTarget - PosY;
			ErT	= TTarget - Theta;
			DErT=ErT-ErT_1;	
			ErT_1=ErT;

			ErD 	= sqrt((ErX * ErX) + (ErY * ErY));
			ErDMax 	= ErD;
/*			
			TT=((ErDMax - ErD)/ErDMax * (TTarget));		

			ErT=TT - Theta;																// HEADINGLOCK

			DErT=ErT-ErT_1;

			ErT_1=ErT;

*/
			ErRes		= ResTarget - ResRobot;	

			DErRes 		= ErRes - ErRes_1;

			ErRes_1 	= ErRes;
			
			ResTarget 		= ((atan2(XTarget-Xn, YTarget-Yn))*RADS);				
			ResRobot		= (atan2(ErX, ErY))*RADS;

			if(ResRobot>0)flagrun=0;
			//PID = (20.0 * ErRes) + (50.0 * DErRes);
								
			PID = (12.0 * ErRes) + (50.0 * DErRes);

			PIDT 		= (0.001 * ErT) + (1.0 * DErT);
			//PIDT 		= (0.005 * ErT) + (0.1 * DErT);


			if(PID>1000)					{PID=1000;}
			else if(PID<-1000)		{PID=-1000;}

			if(PIDT>1000)					{PIDT=1000;}
			else if(PIDT<-1000)		{PIDT=-1000;}			

			ControlY	= (cos((ResTarget -PID) / RADS) * TopY);
			ControlX  	= (sin((ResTarget -PID) / RADS) * TopX);
			ControlT  	= -PIDT * (TopT);


			if(ControlX>=1000) 				ControlX=1000;
			else if(ControlX<=-1000)			ControlX=-1000;
			if(ControlY>=1000) 				ControlY=1000;
			else if(ControlY<=-1000)			ControlY=-1000;
			if(ControlT>=1000) 				ControlT=1000;
			else if(ControlT<=-1000)			ControlT=-1000;

			if(flagrun==1){ControlX=ControlY=ControlT=0;}


		}
		else{
			flagrun=1;
			//runqueue++;
		}

}
void CobaPath(){
	if(runqueue==0){
		FuzzyBP(0,200,0,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}

	if(runqueue==1){
		FuzzyBP(200,200,0,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
		if(runqueue==2){
		FuzzyBP(200,100,0,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}

	if(runqueue==3){
		FuzzyBP(100,0,0,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}

	if(runqueue==4){
		FuzzyBP(0,0,0,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
}

void Path1(){
	if(runqueue==0){
		bicycle_path(141,-18,0,800,R,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}

	if(runqueue==1){
		bicycle_path(610,18,90,800,R,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}

	if(runqueue==2){
		bicycle_path(759,-36,90,800,R,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}

	if(runqueue==3){
		bicycle_path(918,-450,0,800,R,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==4){
		bicycle_path(723,-477,0,800,R,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==5){
		bicycle_path(891,-600,0,800,R,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==6){
		bicycle_path(714,-732,0,800,R,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==7){
		bicycle_path(309,-345,10,800,R,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}	
	if(runqueue==8){
		bicycle_path(75,-654,10,800,R,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==9){
		bicycle_path(-54,-84,10,800,R,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
}

void Path2(){
	if(runqueue==0){
		bicycle_path(150,150,0,800,R,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}

	if(runqueue==1){
		bicycle_path(450,200,90,800,R,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}

	if(runqueue==2){
		bicycle_path(350,400,90,800,R,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}

	if(runqueue==3){
		bicycle_path(-250,400,0,800,R,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==4){
		bicycle_path(-550,300,0,800,R,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==5){
		bicycle_path(-500,150,0,800,R,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==6){
		bicycle_path(-650,-200,0,800,R,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==7){
		bicycle_path(-350,-350,10,800,R,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}	
	if(runqueue==8){
		bicycle_path(-350,-100,10,800,R,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==9){
		bicycle_path(0,-100,10,800,R,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	
}

void Path3(){
	if(runqueue==0){
		bicycle_path(-300,0,0,800,R,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}

	if(runqueue==1){
		bicycle_path(-300,200,90,800,R,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}

	if(runqueue==2){
		bicycle_path(-100,300,90,800,R,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}

	if(runqueue==3){
		bicycle_path(100,300,0,800,R,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==4){
		bicycle_path(300,200,0,800,R,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==5){
		bicycle_path(300,0,0,800,R,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==6){
		bicycle_path(100,-300,0,800,R,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==7){
		bicycle_path(0,-300,10,800,R,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}	
	if(runqueue==8){
		bicycle_path(-300,-100,10,800,R,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==9){
		bicycle_path(0,-100,10,800,R,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
}

void Path3Fuzzy(){
	if(runqueue==0){
		FuzzyBP(-300,0,0,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}

	if(runqueue==1){
		FuzzyBP(-300,200,90,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}

	if(runqueue==2){
		FuzzyBP(-100,300,90,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}

	if(runqueue==3){
		FuzzyBP(100,300,0,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==4){
		FuzzyBP(300,200,0,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==5){
		FuzzyBP(300,0,0,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==6){
		FuzzyBP(100,-300,0,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==7){
		FuzzyBP(0,-300,10,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}	
	if(runqueue==8){
		FuzzyBP(-300,-100,10,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==9){
		FuzzyBP(0,-100,10,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
}

void Path2Fuzzy(){
	if(runqueue==0){
		FuzzyBP(150,150,0,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}

	if(runqueue==1){
		FuzzyBP(450,200,90,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}

	if(runqueue==2){
		FuzzyBP(350,400,90,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}

	if(runqueue==3){
		FuzzyBP(-250,400,0,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==4){
		FuzzyBP(-550,300,0,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==5){
		FuzzyBP(-500,150,0,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==6){
		FuzzyBP(-650,-200,0,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==7){
		FuzzyBP(-350,-350,10,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}	
	if(runqueue==8){
		FuzzyBP(-350,-100,10,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==9){
		FuzzyBP(0,-100,10,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
}

void Path1Fuzzy(){
	if(runqueue==0){
		FuzzyBP(0,0,0,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}

	if(runqueue==1){
		FuzzyBP(0,0,90,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==2){
		FuzzyBP(0,0,0,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}

	if(runqueue==3){
		FuzzyBP(0,0,90,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==4){
		FuzzyBP(0,0,0,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}

	if(runqueue==5){
		FuzzyBP(0,0,90,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==6){
		FuzzyBP(0,0,0,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}

	if(runqueue==7){
		FuzzyBP(0,0,90,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==8){
		FuzzyBP(0,0,0,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}

	if(runqueue==9){
		FuzzyBP(0,0,90,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
/*
	if(runqueue==2){
		FuzzyBP(759,-36,90,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}

	if(runqueue==3){
		FuzzyBP(918,-450,0,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==4){
		FuzzyBP(723,-477,0,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==5){
		FuzzyBP(891,-600,0,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==6){
		FuzzyBP(714,-732,0,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==7){
		FuzzyBP(309,-345,10,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}	
	if(runqueue==8){
		FuzzyBP(75,-654,10,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==9){
		FuzzyBP(-54,-84,10,0.01);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
*/
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "PathPlanning");
  ros::NodeHandle n;
  ros::NodeHandle vel;

  ros::Subscriber sub = n.subscribe("odom", 1000, chatterCallback);
  ros::Subscriber sub_imu = n.subscribe("imu",1000,imuCallback);
  ros::Publisher vel_pub = vel.advertise<geometry_msgs::Twist>("/omnirobot/cmd_vel", 100);

  geometry_msgs::Twist msg;

	
  sudutawal=Theta;
  flags=0;

  ros::Rate r(100);
  int s;
  while(ros::ok){
	i++;k++;

	//flagrun=0;


	//CobaPath();
	FuzzyBP(0,200,0,0.1);
	//Path2Fuzzy();

	if(flagrun==1){
		msg.linear.x = 0;
		msg.linear.y = 0;
		msg.linear.z = 0;

	}

	else{
		InversKinematic(ControlX,ControlY,ControlT);
		msg.linear.x = V3;
		msg.linear.y = V1;
		msg.linear.z = V2;
	}
	
    vel_pub.publish(msg);

    ros::spinOnce();
	r.sleep();
  }

  return 0;
}

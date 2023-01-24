#include "ros/ros.h"
#include "math.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#define RADS	57.2957795f


float PosX,PosY,Theta;
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
float runqueue=0;

void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  PosX = (msg->pose.pose.position.x)*100;
  PosY = (msg->pose.pose.position.y)*100;
  Theta = msg->pose.pose.position.z;
	if(Theta>360)Theta=360;
	else if(Theta<-360)Theta=-360;
  ROS_INFO("Seq: [%d]", msg->header.seq);
  ROS_INFO("Position-> PosX: [%f], PosY: [%f], Theta: [%f]", PosX,PosY,ControlT);
  ROS_INFO("Speed-> ControlY: [%f], CosD: [%f], ControlT: [%f]", ControlY,cosD,ControlT);
  ROS_INFO("Position-> ControlX: [%f], SinD: [%f], Resultan: [%f]", ControlX,sinD,ResTarget);

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

			cosD=cos((sudut_Theta1)/RADS);
			sinD=sin((sudut_Theta1)/RADS);
			//cosD=cos((sudut_Theta1-Theta)/RADS);
			//sinD=sin((sudut_Theta1-Theta)/RADS);
			selisihX=SPX-xa; selisihY=SPY-ya;

		}
		else if(arah==0){
			cosD=cos((sudut_Theta2)/RADS);
			sinD=sin((sudut_Theta2)/RADS);
			//cosD=cos((sudut_Theta2-Theta)/RADS);
			//sinD=sin((sudut_Theta2-Theta)/RADS);
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
		ControlT = (ErT*respon) + (DErT *2);
		

		
	   
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

			PIDT 		= (0.01 * ErT) + (2.0 * DErT);
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


int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry");
  ros::NodeHandle n;
  ros::NodeHandle vel;

  ros::Subscriber sub = n.subscribe("omnirobot/odom", 1000, chatterCallback);
  ros::Publisher vel_pub = vel.advertise<geometry_msgs::Twist>("omnirobot/cmd_vel", 100);
  geometry_msgs::Twist msg;



  while(ros::ok){
	
	if(runqueue==0){
		bicycle_path(0,-250,90,800,10,0);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==1){
		bicycle_path(-250,-250,90,800,10,0);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==2){
		bicycle_path(-250,100,90,800,10,0);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==3){
		bicycle_path(-100,300,90,800,10,0);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==4){
		bicycle_path(0,400,90,800,10,0);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==5){
		bicycle_path(100,400,90,800,10,0);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==6){
		bicycle_path(250,300,90,800,10,0);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==7){
		bicycle_path(100,200,90,800,10,0);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}	
	if(runqueue==8){
		bicycle_path(400,100,90,800,10,0);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==9){
		bicycle_path(0,0,90,800,10,0);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	


	if(flagrun==1){
		msg.linear.x = 0;
		msg.linear.y = 0;
		msg.linear.z = 0;

	}

	else{
		msg.linear.x = ControlX;
		msg.linear.y = ControlY;
		msg.linear.z = ControlT;
	}
    vel_pub.publish(msg);


    ros::spinOnce();
  }

  return 0;
}

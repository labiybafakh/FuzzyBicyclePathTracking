#include "ros/ros.h"
#include "math.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#define RADS	57.2957795f


float PosX,PosY,Theta;
float ErX,ErY,ErT,ErD,DErT,ErT_1,PID,PIDX,PIDY,PIDT,Xn,Yn,Tn,IErRes,ErDX,ErDY,ErDY_1,ErDX_1,ErDT_1;
float Gyro,ControlX,ControlY,ControlT,XX,YY;
float TopX,TopY,TopT,TT,ErDMax,ResTarget,ResRobot,ErRes,DErRes,ErRes_1;
float flagrun=0;
float flags=0;
float runqueue=0;
float rest=0;
void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  PosX = (msg->pose.pose.position.x)*100;
  PosY = (msg->pose.pose.position.y)*100;
  Theta = msg->pose.pose.position.z;
	if(Theta>360)Theta=360;
	else if(Theta<-360)Theta=-360;
  ROS_INFO("Seq: [%d]", msg->header.seq);
  ROS_INFO("Position-> PosX: [%f], PosY: [%f], Theta: [%f]", ControlX,ControlY,ControlT);
  ROS_INFO("Position-> flags: [%f], flagrun: [%f], queue: [%f]", ResTarget,ResRobot,runqueue);

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
		ErT	= TTarget - Theta;


		ErDX	= ErX - ErDX_1;
		ErDY	= ErY - ErDY_1;
		DErT=ErT-ErT_1;

		ErDX_1	= ErDX;
		ErDY_1	= ErDY;
		ErT_1=ErT;
		

		//ResTarget = ((atan2(XTarget-Xn, YTarget-Yn))*RADS);				
		//ResRobot		= (atan2(ErX, ErY))*RADS;
		ResTarget = sqrt((atan2(ErX,ErY)*RADS)*(atan2(ErX,ErY)*RADS)); 
	
		if(ResTarget>6 && flagrun==0){

			ResTarget = sqrt((atan2(ErX,ErY)*RADS)*(atan2(ErX,ErY)*RADS)); 


			ErX 	= XTarget - PosX;
			ErY 	= YTarget - PosY;
			ErT	= TTarget - Theta;

			//ResTarget 		= ((atan2(XTarget-Xn, YTarget-Yn))*RADS);				
			//ResRobot		= (atan2(ErX, ErY))*RADS;

			ErDX	= ErX - ErDX_1;
			ErDY	= ErY - ErDY_1;
			DErT=ErT-ErT_1;

			ErDX_1	= ErDX;
			ErDY_1	= ErDY;
			ErT_1=ErT;
			/*

			TT=((ErDMax - ErD)/ErDMax * (TTarget));		
			ErT=TT - Theta;																// HEADINGLOCK
			DErT=ErT-ErT_1;
			ErT_1=ErT;

			ErRes		= ResTarget - ResRobot;	
			DErRes 		= ErRes - ErRes_1;
			ErRes_1 	= ErRes;
			*/


			if(ResTarget>6)flagrun=0;
			else flagrun=1;
			
			//PID = (0.79f * ErRes) + (0.5 * DErRes);
			PIDX 		= (0.01 * ErX) + (0.005 * ErDX);
			PIDY		= (0.003 * ErY) + (0.009* ErDY);
			PIDT 		= (0.004 * ErT) + (0.004 * DErT);

			//if(PID>1000)					{PID=1000;}
			//else if(PID<-1000)		{PID=-1000;}



			if(PIDX>1000)					{PIDX=1000;}
			else if(PIDX<-1000)		{PIDX=-1000;}

			if(PIDY>1000)					{PIDY=1000;}
			else if(PIDY<-1000)		{PIDY=-1000;}
			
			if(PIDT>1000)					{PIDT=1000;}
			else if(PIDT<-1000)		{PIDT=-1000;}			

			//ControlY	= cos((ResTarget -PID) / RADS) * TopY;
			//ControlX  	= sin((ResTarget -PID) / RADS) * TopX;
			ControlY	= PIDY * TopY;
			ControlX  	= PIDX * TopX;
			//ControlT  	= -PIDT * (TopT);
/*


			if(ControlX>=1000) 				ControlX=1000;
			else if(ControlX<=-1000)			ControlX=-1000;
			if(ControlY>=1000) 				ControlY=1000;
			else if(ControlY<=-1000)			ControlY=-1000;
			if(ControlT>=1000) 				ControlT=1000;
			else if(ControlT<=-1000)			ControlT=-1000;
*/
			if(flagrun==1){ControlX=ControlY=ControlT=0;}



		}
		else{
			flagrun=1;
		}

}

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
	
		if((ErRes>2 || ErRes<-2) && flagrun==0){

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
		GoingTo(-300,0,0,800);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}

	if(runqueue==1){
		GoingTo(-300,200,90,800);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}

	if(runqueue==2){
		GoingTo(-100,300,90,800);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}

	if(runqueue==3){
		GoingTo(100,300,0,800);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==4){
		GoingTo(300,200,0,800);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==5){
		GoingTo(300,0,0,800);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==6){
		GoingTo(100,-300,0,800);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==7){
		GoingTo(0,-300,10,800);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}	
	if(runqueue==8){
		GoingTo(-300,-100,10,800);
		if(flagrun==1){flags=0;runqueue++;}
		else flags=1;
	}
	if(runqueue==9){
		GoingTo(0,-100,10,800);
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

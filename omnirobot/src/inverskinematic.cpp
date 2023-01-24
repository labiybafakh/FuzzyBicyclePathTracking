#include "ros/ros.h"
#include "math.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Int32.h"
#include "stdio.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"


#define RADS	57.2957795f


#define R 110

using namespace std;

float V1,V2,V3;

void V1Callback(const std_msgs::Int32::ConstPtr& dataV1){
	V1 = dataV1->data;
}

void V2Callback(const std_msgs::Int32::ConstPtr& dataV2){
	V2 = dataV2->data;
}

void V3Callback(const std_msgs::Int32::ConstPtr& dataV3){
	V3 = dataV3->data;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "PathPlanning");
  ros::NodeHandle n;
  ros::NodeHandle vel;

  ros::Subscriber sub1 = n.subscribe("V1", 1000,V1Callback);
  ros::Subscriber sub2 = n.subscribe("V2",1000,V2Callback);
  ros::Subscriber sub3 = n.subscribe("V3",1000,V3Callback);
  ros::Publisher vel_pub = vel.advertise<geometry_msgs::Twist>("/omnirobot/cmd_vel", 100);

  geometry_msgs::Twist msg;


  ros::Rate r(100);
  int s;
  while(ros::ok){

   printf("\n%.2f\t%.2f\t%.2f",V1,V2,V3);

    ros::spinOnce();
	r.sleep();
  }

  return 0;
}

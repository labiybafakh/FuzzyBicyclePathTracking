#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#define Clicks1 0.10f  
#define Clicks2 0.24f 
#define rads 57.2958f
 
ros::NodeHandle  nh;
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

double L1=0.09,L2=0.097,L3=0.09/7;
double Rwheels = 0.048;
double x = 0.0;
double y = 0.0;
double theta = 0;
double pi = 3.1415926535897932384626433832795028841971693993751058209749445923078164062; 
char base_link[] = "/base_link";
char odom[] = "/odom";

char buff[33];
double Count1=0,Count2=0,Count3=0;
double PosX=0,PosY=0,Theta=0;
int32 Distance1=0,Distance2=0,Distance3=0,T_Rad;
int32 LastTim1,LastTim3,LastTim4;
int32 XR,YR,TR,Heading;
double VR[4],RpsRotary[4];
double VX=0,VY=0,VT=0;

void VRotary(){

  //Ambil Data Encoder
   Count1 = Timer1.getCount()-32267;
   Count2 = Timer3.getCount()-32267;
   Count3 = Timer4.getCount()-32267;  

   Timer1.setCount(32267);
   Timer3.setCount(32267);
   Timer4.setCount(32267);

   //Konversi Ke RPS
   RpsRotary[1] = Count1*100 / 720; 
   RpsRotary[2] = Count2*100 / 720;
   RpsRotary[3] = Count3*100 / 720;

   //Konversi RPS ke Kecepatan Linier
   VR[1] = RpsRotary[1] * pi * Rwheels;
   VR[2] = RpsRotary[2] * pi * Rwheels;
   VR[3] = RpsRotary[3] * pi * Rwheels;


//  VX = VR[1] - VR[2]*0.5f - VR[3]*0.5f;
//  VY = VR[3]*0.866 - VR[2]*0.866;   
//  VT = VR[1]/L1 + VR[2]/L2 + VR[3]/L3;
//
//  PosX += VX*0.01;
//  PosY += VY*0.01;
//  Theta+=VT*0.01;

   VX =  -0.6667*VR[1] + VR[2]*0.333 + VR[3]*0.333;
   VY = VR[3]*0.5774 - VR[2]*0.5774;
   VT = VR[1]/(3*L1) + VR[2]/(3*L2) + VR[3]/(3*L3);

  PosX += VX*0.01;
  PosY += VY*0.01;
  Theta+=VT*0.01;

  
   Count1 = 0;
   Count2 = 0;
   Count3 = 0;
}



void Odometri(){

  VX = VR[1] - VR[2]*0.5f + VR[3]*0.5f;
  VY = VR[3]*0.866 - VR[2]*0.866;
  VT = VR[3];

  PosX += VX*0.1;
  PosY += VY*0.1;
  Theta+=VT*0.1;
//  XR = Distance3 - Distance1*0.5 - Distance2*0.5;
//  YR = Distance1*0.866 - Distance2*0.866;
//  TR = Distance1/L + Distance2/L + Distance3/L;
//  
//  Heading = (float)100;
//  if(Heading==0)  T_Rad=0 - 0.038;
//  else            T_Rad=(double)(Heading/57.32) - 0.038;   
//  PosX=(double) XR * cos(T_Rad) + (double)YR * sin(T_Rad);
//  PosY=(double) YR * cos(T_Rad) - (double)XR * sin(T_Rad);
}

void ResetEncoder(){

      Timer1.setCount(32267);Timer3.setCount(32267);Timer4.setCount(32267);delay(100);       
      Count1=0;
      Count2=0;
      Count3=0;
      PosX=0;
      PosY=0;
      Theta=0;
     
}




void setup()
{
//    Serial.begin(115200); // Ignored by Maple. But needed by boards using hardware serial via a USB to Serial adaptor

    pinMode(PA6, INPUT_PULLUP);  //channel A
    pinMode(PA7, INPUT_PULLUP);  //channel B

    pinMode(PA8, INPUT_PULLUP);  //channel A
    pinMode(PA9, INPUT_PULLUP);  //channel B

    pinMode(PB6, INPUT_PULLUP);  //channel A
    pinMode(PB7, INPUT_PULLUP);  //channel B

    ResetEncoder();

    Timer2.setMode(TIMER_CH1, TIMER_OUTPUTCOMPARE);
    Timer2.setPeriod(10000);                           // in microseconds
    Timer2.setCompare(TIMER_CH1, 1);                  // overflow might be small
    Timer2.attachInterrupt(TIMER_CH1, VRotary);

    Timer1.setMode(12, TIMER_ENCODER);                Timer3.setMode(12, TIMER_ENCODER);                Timer4.setMode(12, TIMER_ENCODER);
    Timer1.pause();                                   Timer3.pause();                                   Timer4.pause();
    Timer1.setEdgeCounting(TIMER_SMCR_SMS_ENCODER3);  Timer3.setEdgeCounting(TIMER_SMCR_SMS_ENCODER3);  Timer4.setEdgeCounting(TIMER_SMCR_SMS_ENCODER3);
    Timer1.setOverflow(65535);                        Timer3.setOverflow(65535);                        Timer4.setOverflow(65535);
    Timer1.resume();                                  Timer3.resume();                                  Timer4.resume();

    ResetEncoder();
    delay(1000);
    nh.initNode();
    broadcaster.init(nh);
   
}

void loop() {

    //ReadEncoder();
    //Odometri();    
    sprintf(buff,"%.2f\t%.2f\t%.2f",PosX,PosY,Theta);
    Serial.println(buff);

//    odo.header.frame_id = odom;
//    odo.child_frame_id = base_link;
//    odo.header.stamp = nh.now();
//
//  
//    odo.pose.pose.position.x = PosX;
//    odo.pose.pose.position.y = PosY;
//    odo.pose.pose.position.z = Theta;
//
//    
//    odo.twist.twist.linear.x = VR[1];
//    odo.twist.twist.linear.y = VR[2];
//    odo.twist.twist.angular.z = VR[3];
//  
//
//    odom_pub.publish(&odo);
//    nh.spinOnce();

//    // tf odom->base_link
//    t.header.frame_id = odom;
//    t.child_frame_id = base_link;
//    
//    t.transform.translation.x = PosX;
//    t.transform.translation.y = PosY;
//    
//    t.transform.rotation = tf::createQuaternionFromYaw(Theta);
//    t.header.stamp = nh.now();
//    
//    broadcaster.sendTransform(t);
//    nh.spinOnce();
//    
    delay(10);
}


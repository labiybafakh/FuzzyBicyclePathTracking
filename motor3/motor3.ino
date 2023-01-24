#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
//
#define PwmMotor  PA3
#define Enable    PA5
#define Direction PA4
#define EncA      PB6
#define EncB      PB7

//double kp=1.1,ki=20.4,kd=0.000001;
double kp=0.5,ki=7.4,kd=0.000001;
double Freq=0, RPM=0;
int32 PID=0;
double samplingRot, lastRotary, Rotary;
double Error, Derror, Ierror, LastError;
int32 SetRPM=0;
int tik;
char buff[33];
int32 Count1 = 0, Count2 = 0, Count3 = 0;
int32 PosX, PosY, Theta;
int32 Distance1 = 0, Distance2 = 0, Distance3 = 0, T_Rad;
int32 LastTim1, LastTim3, LastTim4;
int32 XR, YR, TR, Heading;
int32 Sampling, Encoders, LastEncoders;//, RPM = 0;

void callback_vmotor(const geometry_msgs::Twist& msg){
     SetRPM = (int32) msg.linear.z;
}

ros::NodeHandle  nh;
ros::NodeHandle  subh;
std_msgs::Int32 vmotor;
ros::Publisher vpub("V3", &vmotor);
ros::Subscriber<geometry_msgs::Twist> sub("/omnirobot/cmd_vel",callback_vmotor);



int32 ReadEncoder() {

  Count3 = Timer4.getCount() - 32267;
  Timer4.setCount(32267);
  Sampling += Count3;
  Count3 = 0;

  return Sampling;
}

void ResetEncoder() {
  Count1 = 0;
  Count2 = 0;
  Count3 = 0;
  PosX = 0;
  PosY = 0;
  Theta = 0;
//  Timer1.setCount(65535); Timer3.setCount(65535); Timer4.setCount(65535); delay(100);
//  Timer1.setCount(32267); Timer3.setCount(32267); Timer4.setCount(32267); delay(100);

}



void setup()
{
  //Serial.begin(115200); // Ignored by Maple. But needed by boards using hardware serial via a USB to Serial adaptor

  pinMode(EncA, INPUT_PULLUP);  //channel A
  pinMode(EncB, INPUT_PULLUP);  //channel B
  pinMode(Direction, OUTPUT);
  pinMode(Enable, OUTPUT);
  pinMode(PwmMotor, PWM);
  pwmWrite(PwmMotor,0);
  
    Timer2.pause();
    Timer2.setPrescaleFactor(72); // 1 µs resolution
    Timer2.setCompare(TIMER_CH3, 0);
    Timer2.setOverflow(1000);
    Timer2.refresh();
    Timer2.resume(); // let timer 3 run


  Timer1.setMode(TIMER_CH1, TIMER_OUTPUTCOMPARE);
  Timer1.setPeriod(10000);                           // in microseconds
  Timer1.setCompare(TIMER_CH1, 1);                  // overflow might be small
  Timer1.attachInterrupt(TIMER_CH1, motorRPM);

  Timer4.setMode(0, TIMER_ENCODER);                   //set mode, the channel is not used when in this mode.
  Timer4.pause();                                     //stop...
  Timer4.setPrescaleFactor(1);                        //normal for encoder to have the lowest or no prescaler.
  Timer4.setOverflow(65535);                          //use this to match the number of pulse per revolution of the encoder. Most industrial use 1024 single channel steps.
  Timer4.setCount(32267);
  Timer4.setEdgeCounting(TIMER_SMCR_SMS_ENCODER3);    //or TIMER_SMCR_SMS_ENCODER1 or TIMER_SMCR_SMS_ENCODER2. This uses both channels to count and ascertain direction.
  Timer4.resume();    
  
  delay(1000);
  
  nh.initNode();
  nh.advertise(vpub);
  nh.subscribe(sub);

}


void loop() {
  vmotor.data = SetRPM;
  vpub.publish(&vmotor);

  nh.spinOnce();
  
  delay(10);
}

void motorRPM()
{  

    Rotary = Timer4.getCount() - 32267;
    Timer4.setCount(32267);

    Freq         = Rotary * 100;
    RPM          = Freq * 60 / 270; //268 PPR mode 0

    Error = SetRPM - RPM;
          
    Ierror  += Error * 0.01;
    Derror  = (Error - LastError)/0.01;
    PID     = (int)((kp * Error) + (kd * Derror) + (ki*Ierror)) ;

    lastRotary = Rotary;
    LastError = Error;

    if (PID >= 1000) PID = 1000;
    else if(PID <= -1000) PID = -1000;
    
    motor(PID);

    Rotary = 0;
}

void motor(int kecepatan) {
  int pwmPositif;
  int pwmNegatif;

  digitalWrite(Enable, HIGH);

  if (kecepatan > 0) {
    pwmPositif = kecepatan;
    digitalWrite(Direction, LOW);
    pwmWrite(PwmMotor, pwmPositif);
  }
  else if (kecepatan < 0) {
    pwmNegatif = 1000 + kecepatan;
    digitalWrite(Direction, HIGH);
    pwmWrite(PwmMotor, pwmNegatif);
  }
  else {
    digitalWrite(Direction, LOW);
    pwmWrite(PwmMotor, 0);
  }
}

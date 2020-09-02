#if (ARDUINO >= 100)
    #include <Arduino.h>
#else
    #include <WProgram.h>
#endif

#include <Servo.h>

#include "ros.h"
#include "ros/time.h"
//header file for publishing velocities for odom
#include "lino_msgs/Velocities.h"
//header file for cmd_subscribing to "cmd_vel"
#include "geometry_msgs/Twist.h"
//header file for pid server
//#include "lino_msgs/PID.h"

#include "lino_base_config.h"
#include "Motor.h"
#include "Kinematics.h"
//#include "PID.h"
#include <FreqMeasureMulti.h>

// Measure 3 frequencies at the same time! :-)
FreqMeasureMulti freqL;
FreqMeasureMulti freqR;


#define COMMAND_RATE 20 //hz
#define DEBUG_RATE 5

Controller motor1_controller(Controller::MOTOR_DRIVER, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Controller motor2_controller(Controller::MOTOR_DRIVER, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B); 

//PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
//PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

#define DEBUG 0


Kinematics kinematics(Kinematics::LINO_BASE, MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);

float g_req_linear_vel_x = 0;
float g_req_linear_vel_y = 0;
float g_req_angular_vel_z = 0;

unsigned long g_prev_command_time = 0;

float sum1=0, sum2=0;
int count1=0, count2=0;
long xlf=0, xrf=0, lf =0, rf = 0;
bool rev[2] = {false, false};
elapsedMillis timeout;

//callback function prototypes
void commandCallback(const geometry_msgs::Twist& cmd_msg);
//void PIDCallback(const lino_msgs::PID& pid);

ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);
//ros::Subscriber<lino_msgs::PID> pid_sub("pid", PIDCallback);

lino_msgs::Velocities raw_vel_msg;
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);

void setup()
{
  
    nh.initNode();
    nh.getHardware()->setBaud(57600);
//    nh.subscribe(pid_sub);
    nh.subscribe(cmd_sub);
    nh.advertise(raw_vel_pub);

    delay(10);
    freqL.begin(MOTOR1_ENCODER_A);
    freqR.begin(MOTOR2_ENCODER_A);

    while (!nh.connected())
    {
        nh.spinOnce();
    }
    nh.loginfo("LINOBASE CONNECTED");
    delay(1);
}

void loop()
{
    static unsigned long prev_control_time = 0;
    static unsigned long prev_debug_time = 0;
  if (freqL.available()) {
	  sum1 = sum1 + freqL.read();
    count1 = count1 + 1;
  }
  if (freqR.available()) {
	sum2 = sum2 + freqR.read();
    count2 = count2 + 1;
  }
if (timeout > 500) {
  if (count1 > 0) {
      lf = (freqL.countToFrequency(sum1 / count1));
    } else {
      lf = 0;
    }
    if (count2 > 0) {
    rf = (freqR.countToFrequency(sum2 / count2));
    } else {
      rf = 0;
    }
    sum1 = 0;
    sum2 = 0;
    count1 = 0;
    count2 = 0;
    timeout = 0;
}


    //this block drives the robot based on defined rate
    if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE))
    {
        moveBase();
        prev_control_time = millis();
    }

    //this block stops the motor when no command is received
    if ((millis() - g_prev_command_time) >= 400)
    {
        stopBase();
    }


    //this block displays the encoder readings. change DEBUG to 0 if you don't want to display
    if(DEBUG)
    {
        if ((millis() - prev_debug_time) >= (1000 / DEBUG_RATE))
        {
            printDebug();
            prev_debug_time = millis();
        }
    }
    //call all the callbacks waiting to be called
    nh.spinOnce();
}

//void PIDCallback(const lino_msgs::PID& pid)
//{
//    //callback function every time PID constants are received from lino_pid for tuning
//    //this callback receives pid object where P,I, and D constants are stored
//    motor1_pid.updateConstants(pid.p, pid.i, pid.d);
//    motor2_pid.updateConstants(pid.p, pid.i, pid.d);
//}

void commandCallback(const geometry_msgs::Twist& cmd_msg)
{
    //callback function every time linear and angular speed is received from 'cmd_vel' topic
    //this callback function receives cmd_msg object where linear and angular speed are stored
    g_req_linear_vel_x = cmd_msg.linear.x;
    g_req_linear_vel_y = cmd_msg.linear.y;
    g_req_angular_vel_z = cmd_msg.angular.z;

    g_prev_command_time = millis();
}

void moveBase()
{
    //get the required rpm for each motor based on required velocities, and base used
    Kinematics::rpm req_rpm = kinematics.getRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);

  
    //get the current speed of each motor
    int current_rpm1 = (rev[0]?-lf/COUNTS_PER_REV:lf/COUNTS_PER_REV);
	int current_rpm2 = (rev[1]?-rf/COUNTS_PER_REV:rf/COUNTS_PER_REV);	
   // int current_rpm2 = rf/COUNTS_PER_REV;
  
    xlf = current_rpm1;
    xrf = current_rpm2;


    //the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
    //the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
    if (req_rpm.motor1 <=0) {rev[0]=true;}
	else {rev[0]=false;}
	if (req_rpm.motor2 <=0) {rev[1]=true;}
	else {rev[1]=false;}
    motor1_controller.spin(1,req_rpm.motor1);
    motor2_controller.spin(2,req_rpm.motor2); 
    
    
      

    Kinematics::velocities current_vel;

    current_vel = kinematics.getVelocities(current_rpm1, current_rpm2);
       
    //pass velocities to publisher object
    raw_vel_msg.linear_x = current_vel.linear_x;
    raw_vel_msg.linear_y = current_vel.linear_y;
    raw_vel_msg.angular_z = current_vel.angular_z;

    //publish raw_vel_msg
    raw_vel_pub.publish(&raw_vel_msg);
}

void stopBase()
{
    g_req_linear_vel_x = 0;
    g_req_linear_vel_y = 0;
    g_req_angular_vel_z = 0;
}


float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void printDebug()
{
    char buffer[50];

    sprintf (buffer, "Encoder Left  : %ld", xlf);
    nh.loginfo(buffer);
    sprintf (buffer, "Encoder Right : %ld", xrf);
    nh.loginfo(buffer);
  
}


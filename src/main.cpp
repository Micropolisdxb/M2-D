/********************************************************************************/
/*   Author  : Ahmed Farag  & Wajih                                             */
/*   Date    : 18/05/2022                                                       */
/*   Version : V02                                                              */
/********************************************************************************/

// Includes
#include "SBUS.h"
#include <Arduino.h>
#include <T_Motor.h>
#include <Curtis_Configuration.h>
#include <Brake.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>

uint16_t channels[16];
int Potentiometer_Readings = 0;
bool failSafe;
bool lostFrame;
char c;
int RC_Readings;
int RC_Switch;
int RC_Key_Switch_Reading;
int RC_Steering_Val;
int Throttle_Value;
int ROS_Steering_Val = 0;
int ROS_Enable = LOW;

int ROS_Emergency_Brake = LOW;

// Create node handle to manage pub and sub
ros::NodeHandle nh;

void Steering_cb(const std_msgs::Int16 &Steering_msg)
{
  ROS_Steering_Val = -(Steering_msg.data);
  nh.loginfo(("Steering Value: " + String(ROS_Steering_Val)).c_str());
  digitalWrite(LED_BUILTIN, HIGH - digitalRead(LED_BUILTIN)); // blink the led
}

void throttle_cb(const std_msgs::Int16 &throttlevel)
{
  Throttle_Value = throttlevel.data;
  nh.loginfo((String("Throttle_value = ") + String(Throttle_Value).c_str()).c_str());
}
void Emergency_Brake_cb(const std_msgs::Bool &Emergency_Brake_msg)
{
  ROS_Emergency_Brake = Emergency_Brake_msg.data;

  nh.loginfo((String("Throttle_value = ") + String(Throttle_Value).c_str()).c_str());
}
// Creating a ros subscriber for cmd_vel topic
ros::Subscriber<std_msgs::Int16> throttle_sub("throttle_vel", &throttle_cb);
ros::Subscriber<std_msgs::Int16> Steering_Sub("ROS_Steering", &Steering_cb);
ros::Subscriber<std_msgs::Bool> Emergency_Brake_Sub("Emergency_Brake", &Emergency_Brake_cb);

// a SBUS object, which is on hardware
// serial port 1
SBUS x8r(Serial1);

void setup()
{
  // initialization functions
  nh.initNode();
  Steering_Init();
  Curtis_Init();
  Brake_init();

  // Subscriber node for cmd vel topic
  nh.subscribe(throttle_sub);
  nh.subscribe(Steering_Sub);
  nh.subscribe(Emergency_Brake_Sub);

  // begin the SBUS communication
  x8r.begin();

  // RC_Key_Switch_Reading = 1715;
  Serial.println("RC_Key_Switch_Reading = : " + String(RC_Key_Switch_Reading));

  // begin the serial communication
  Serial.begin(115200);
}
int x = 0;
void loop()
{
  Can3.events();
  if (x8r.read(&channels[0], &failSafe, &lostFrame))
    ;
  // Serial.println("FailSafe = : " + String(failSafe));
  // Serial.println("RC_Key_Switch_Reading = : " + String(RC_Key_Switch_Reading));

  if (!failSafe)
  {
    // }
    RC_Readings = channels[RC_FW_CH_NUM];
    RC_Switch = channels[RC_ON_OFF_SW_CH_NUM];
    RC_Key_Switch_Reading = channels[RC_KEY_SW_CH_NUM];
    RC_Steering_Val = channels[STEERING_CH];
    // ROS_Enable = channels[ROS_CH] == 278 ? LOW : HIGH;
  }
  else
    RC_Key_Switch_Reading = 278;

  if (RC_Key_Switch_Reading == 1715)
  {
    Brake_Control_RC(RC_Readings);
    RC_Control_Steps(RC_Steering_Val);
    Curtis_RC(RC_Readings, RC_Switch, RC_Key_Switch_Reading);
    nh.loginfo("RC Control");
  }
  // Serial.println("RC Control"); if (failSafe || RC_Key_Switch_Reading == 278)
  else
  {
    //   Serial.println("ROS Control");

    // ROS PART
    Curtis_Ros(Throttle_Value);
    ROS_Control_Steps(ROS_Steering_Val);
    Brake_Control_ROS(Throttle_Value);
    nh.loginfo("ROS Control");
  }
  nh.spinOnce();
  // Serial.println("---------------");

  //  Serial_Control();

  delayMicroseconds(50);
}

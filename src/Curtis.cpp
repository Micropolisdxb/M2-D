/********************************************************************************/
/*   Author  : Ahmed Farag  & Wajih                                             */
/*   Date    : 18/05/2022                                                       */
/*   Version : V02                                                              */
/********************************************************************************/
#include <Arduino.h>
#include <Curtis_Configuration.h>
#include "SBUS.h"

// Intitialization function
void Curtis_Init()
{
  // Set the PWM pin for the throttle to output
  pinMode(THROTTLE_FORWARD_PIN, OUTPUT);
  // Set the Forward switch pin  to output
  pinMode(FORWARD_DIRECTION_SWITCH, OUTPUT);
  // Set the Reverse switch pin  to output
  pinMode(REVERSE_DIRECTION_SWITCH, OUTPUT);
  // Set the Key switch pin  to output
  pinMode(KEY_SWITCH_PIN, OUTPUT);
  // Set the reverse pin switch to output
  pinMode(REVERSE_DIRECTION_SWITCH, OUTPUT);
  // Set the High brake pin switch to output
  pinMode(HIGH_BRAKE_PIN, OUTPUT);
  // Set the potentiometer pin to input
  pinMode(POTENTIOMETER_PIN_NUMBER, INPUT);
  // Set Reverse pin to low
  digitalWrite(REVERSE_DIRECTION_SWITCH, LOW);
  // Set the spedometer pin to input
  // pinMode(SPEDOMETER_PIN, INPUT);
  digitalWrite(KEY_SWITCH_PIN , HIGH);
}

// Function to get the Potentiomete readings
int Get_Potentiometer_Readings(int Potentiometer_Pin_Num)
{

  int Potentiometer_Readings = analogRead(Potentiometer_Pin_Num);
  return Potentiometer_Readings;
}

// Function to move the motor forward with PWM maped from Potentiometer values
void Curtis_Forward(int Potentiometer_Readings, int Throttle_Pin, int Throttle_Min_Readings, int Throttle_Max_Readings, int Throttle_Min_PWM, int Throttle_Max_PWM)
{
  int Throttle_PWM_Value = map(Potentiometer_Readings, Throttle_Min_Readings, Throttle_Max_Readings, Throttle_Min_PWM, Throttle_Max_PWM);
  Serial.print("Forward PWM ");
  Serial.println(Throttle_PWM_Value);

  digitalWrite(REVERSE_DIRECTION_SWITCH, LOW);
  digitalWrite(FORWARD_DIRECTION_SWITCH, HIGH);
  analogWrite(Throttle_Pin, Throttle_PWM_Value);
}

// Function for the RC control for the driver
void Curtis_RC (int RC_Readings, int RC_Switch, int RC_Key_Switch_Reading)
{
  int RC_Key_Switch = 0;


    if (RC_Key_Switch_Reading == 278)
  {
    RC_Key_Switch = LOW;
  }
  else if (RC_Key_Switch_Reading == 1715)
  {
    RC_Key_Switch = HIGH;
  }

  Curtis_Key_Switch(KEY_SWITCH_PIN, RC_Key_Switch);

  if (RC_Switch == 278)
  {
    if ((RC_Readings >= 1004) && (RC_Readings <= 1722))
    {
      Serial.print("Forward: ");
      Curtis_Forward(RC_Readings, THROTTLE_FORWARD_PIN, RC_THROTTLE_MIN_READINGS, RC_THROTTLE_MAX_READINGS, THROTTLE_MIN_PWM, THROTTLE_MAX_PWM);
    }

    if ((RC_Readings > 1000) && (RC_Readings < 1004))
    {

      analogWrite(THROTTLE_FORWARD_PIN, 0);
      digitalWrite(REVERSE_DIRECTION_SWITCH, LOW);
      digitalWrite(FORWARD_DIRECTION_SWITCH, LOW);
    }

    if ((RC_Readings >= 283) && (RC_Readings <= 1000))
    {

      Serial.print("Reverse: ");
      Curtis_Reverse(RC_Readings, THROTTLE_REVERSE_PIN, RC_THROTTLE_MIN_READINGS_REVERSE, RC_THROTTLE_MAX_READINGS_REVERSE, THROTTLE_MIN_PWM_REVERSE, THROTTLE_MAX_PWM_REVERSE, REVERSE_DIRECTION_SWITCH);
    }
  }
}

// Function to move the motor backward with PWM maped from Potentiometer values
void Curtis_Reverse(int Potentiometer_Readings, int Throttle_Pin, int Throttle_Min_Readings, int Throttle_Max_Readings, int Throttle_Min_PWM, int Throttle_Max_PWM, int Reverse_Switch_Pin)
{
  // Set reverse switch on

  int Throttle_PWM_Value_Rev = map(Potentiometer_Readings, Throttle_Min_Readings, Throttle_Max_Readings, Throttle_Min_PWM, Throttle_Max_PWM);
  Serial.print("Reverse PWM ");
  Serial.println(Throttle_PWM_Value_Rev);

  digitalWrite(FORWARD_DIRECTION_SWITCH, LOW);
  digitalWrite(Reverse_Switch_Pin, HIGH);
  analogWrite(Throttle_Pin, abs(Throttle_PWM_Value_Rev));
}

// Function to activate the high brake
void Curtis_Brake(int Brake_Pin, int State)
{
  digitalWrite(Brake_Pin, State);
}

// Function to activate and disable the key
void Curtis_Key_Switch(int Key_Pin, int Key_State)
{
  digitalWrite(Key_Pin, Key_State);
}

// Function for the RC control for the driver
void Curtis_Ros(int Throttle_Readings)
{

  if (Throttle_Readings > 0)
  {
    Serial.print("Forward: \n");
    Curtis_Forward(Throttle_Readings, THROTTLE_FORWARD_PIN, ROS_THROTTLE_MIN_READINGS, ROS_THROTTLE_MAX_READINGS, THROTTLE_MIN_PWM, THROTTLE_MAX_PWM);
      
  }

  else if (Throttle_Readings == 0)
  {

    analogWrite(THROTTLE_FORWARD_PIN, 0);
    digitalWrite(REVERSE_DIRECTION_SWITCH, LOW);
    digitalWrite(FORWARD_DIRECTION_SWITCH, LOW);
  }

  else if (Throttle_Readings < 0)
  {

    Serial.print("Reverse: ");
    Curtis_Reverse(Throttle_Readings, THROTTLE_REVERSE_PIN, ROS_THROTTLE_MIN_READINGS_REVERSE, ROS_THROTTLE_MAX_READINGS_REVERSE, THROTTLE_MIN_PWM_REVERSE, THROTTLE_MAX_PWM_REVERSE, REVERSE_DIRECTION_SWITCH);
  }
}

// Function for speedo meter for feedback
// int  Votol_Speedometer(int Sedometer_Pin , int Mode)
// {

//   unsigned long Duration_In_Micro = pulseIn(Sedometer_Pin , Mode);

//   if(Duration_In_Micro > 1)
//   {
//   Serial.println(Duration_In_Micro);
//   delay(1000);
//   }

// }

/********************************************************************************/
/*   Author  : Ahmed Farag  & Wajih                                             */
/*   Date    : 18/05/2022                                                       */
/*   Version : V02                                                              */
/********************************************************************************/

#ifndef CURTIS_CONFIGURATION_H
#define CURTIS_CONFIGURATION_H


// Inputs pins 
#define POTENTIOMETER_PIN_NUMBER 22

//Remote control parameters  
  
#define RC_FW_CH_NUM                 2
#define RC_RV_CH_NUM                 2
#define RC_ON_OFF_SW_CH_NUM          7
#define RC_KEY_SW_CH_NUM             8

#define RC_FW_UPPER_LIMIT               1722
#define RC_DEFAULT_LIMIT                1002
#define RC_RV_LOWER_LIMIT               283

#define RC_SWITCH_UP                  278
#define RC_SWITCH_DOWN                1715



// Transistors Pin number 


// Forward motor driver parameters

#define THROTTLE_FORWARD_PIN        36
#define FORWARD_DIRECTION_SWITCH    10
#define THROTTLE_MIN_PWM            0
#define THROTTLE_MAX_PWM            150
#define ROS_THROTTLE_MIN_READINGS       0
#define ROS_THROTTLE_MAX_READINGS       100

#define RC_THROTTLE_MIN_READINGS       1002
#define RC_THROTTLE_MAX_READINGS       1722

// REVERSE motor driver parameters

#define THROTTLE_REVERSE_PIN            36
#define REVERSE_DIRECTION_SWITCH        11 
#define THROTTLE_MIN_PWM_REVERSE       -150
#define THROTTLE_MAX_PWM_REVERSE        0
#define ROS_THROTTLE_MIN_READINGS_REVERSE  -100
#define ROS_THROTTLE_MAX_READINGS_REVERSE   0

#define RC_THROTTLE_MIN_READINGS_REVERSE  283
#define RC_THROTTLE_MAX_READINGS_REVERSE   1002
// HIGH Brake PINS
#define HIGH_BRAKE_PIN           37
#define HIGH_BRAKE_OFF_STATE     LOW
#define HIGH_BRAKE_ON_STATE      HIGH

// Spedometer PINS
#define SPEDOMETER_PIN                 
#define SPEDOMETER_PULSE_MODE_HIGH    HIGH 

// Key switch pin
#define KEY_SWITCH_PIN  12

// Stering channel
#define STEERING_CH     0

// ROS channel
#define ROS_CH     11


// Functions Prototype

void Curtis_Init();
int  Get_Potentiometer_Readings (int Potentiometer_Pin_Num) ;
void Curtis_Forward (int Throttle_Pin , int Potentiometer_Readings , int Throttle_Min_Readings, int Throttle_Max_Readings, int Throttle_Min_PWM ,int Throttle_Max_PWM );
void Curtis_Reverse(int Throttle_Pin , int Potentiometer_Readings , int Throttle_Min_Readings, int Throttle_Max_Readings, int Throttle_Min_PWM ,int Throttle_Max_PWM, int Reverse_Switch_Pin);
void Curtis_Brake(int Brake_Pin , int State);
int  Curtis_Speedometer(int Sedometer_Pin , int Mode);
void Curtis_Key_Switch (int Key_Pin , int Key_State);
void Curtis_RC (int RC_Readings, int RC_Switch, int RC_Key_Switch_Reading);
void Curtis_Ros (int Throttle_vel);



#endif
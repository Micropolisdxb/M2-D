#ifndef T_Motor_h
#define T_Motor_h
#include <FlexCAN_T4.h>


extern FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

// Value Limits
#define P_MIN   -95.5f
#define P_MAX   95.5f
#define V_MIN   -25.64f
#define V_MAX   25.64f
#define KP_MIN  0.0f
#define KP_MAX  500.0f
#define KD_MIN  0.0f
#define KD_MAX  5.0f
#define T_MIN   -18.0f
#define T_MAX   18.0f

// Set values  ID1
#define P_SteeringZero  0.0       // Initial Zero position
#define P_SteeringMin   -55.0f    // Minimum position CCW
#define P_SteeringMax   55.0f     // Max Position CW
#define Steering_Can_ID 0x1        // to be checked

#define v_in    0.0f
#define kp_in   100.0f // Gain
#define kd_in   5.0f  // Derivative
#define t_in    0.0f

#define Encoder_Pin 20  // set encoder pin
#define Min_Angle 200   // set min
#define Max_Angle 600   // set max
#define Zero_Angle 400  // set zero

#define RC_Steer_Min 283    // set min
#define RC_Steer_Max 1722   // set max
#define RC_Steer_Zero 1002  // set zero


void Steering_Init();
void Serial_Control();
void RC_Control(int RC_Val);
void RC_Control_Steps(int RC_Val);
void ROS_Control_Steps(int ROS_ANGLE);




#endif
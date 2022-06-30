#ifndef Brake_h
#define Brake_h

// to be changed
#define Min_LS_Pin 34
#define Max_LS_Pin 41

//Cytron
#define Brake_Dir 32
#define Brake_PWM 29

#define RC_Throttle_Zero   1002
#define RC_Throttle_Max    1722
#define RC_Throttle_Min    283

#define ROS_Throttle_Zero   0
#define ROS_Throttle_Max    100
#define ROS_Throttle_Min    -100


#define RC_E_
void Brake_init();
// void Brake_Control(int Throttle_Value);
void Brake_Control_ROS(int Throttle_Value);
void Brake_Control_RC(int Throttle_Value);



void Brake_Control_Serial();




#endif
#include <T_Motor.h>
#include <Arduino.h>

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;
void canSniff(const CAN_message_t &msg);
void pack_cmd(int ID, float &position);
void enableMode(int ID);
void Exit(int ID);
int Get_Pos(int Encoder_pin);

float P_Steering = 0.0f; // Initial position
boolean GoalState = LOW;
void Steer_Position_Steps(float &Position, int Step_goal);

void Steering_Init()
{
    Can3.begin();
    Can3.setBaudRate(1000000);
    Can3.setMaxMB(16);
    Can3.enableFIFO();
    Can3.enableFIFOInterrupt();
    Can3.onReceive(canSniff);
    Can3.mailboxStatus();
    Can3.enhanceFilter(MB6);

    pinMode(Encoder_Pin, INPUT);

    enableMode(Steering_Can_ID);
    // Steer_Position_Steps(P_Steering, P_SteeringZero);

    Serial.println("Steering Initialized");
}

void Steer_Position(float &Position, int Pos_goal)
{
    int Encoder_val = Get_Pos(Encoder_Pin);
    float error = 0.0025;

    Serial.print("Pos_goal:");
    Serial.println(Pos_goal);

    Pos_goal = Zero_Angle;

    float UpperLimit = Pos_goal + error * Pos_goal;
    float LowwerLimit = Pos_goal - error * Pos_goal;

    Serial.printf("UpperLimit: %f \n", UpperLimit);
    Serial.printf("LowwerLimit: %f \n", LowwerLimit);
    Serial.printf("Encoder_val: %d \n", Encoder_val);
    Serial.printf("Position: %f \n", Position);

    float steps = 1;

    if (Encoder_val > Pos_goal)
    {
        Position += steps;

        Serial.println("left");
        delay(75);
    }

    else if (Encoder_val < Pos_goal)
    {
        Position -= steps;

        Serial.println("right");
        delay(75);
    }
    else if (Encoder_val >= Pos_goal || Encoder_val <= Pos_goal)
    {
        GoalState = LOW;
        // test to rptate to the desired position directly but at low speed
    }

    pack_cmd(Steering_Can_ID, Position);
}

void ROS_Control_Steps(int ROS_ANGLE)
{

    float ROS_Steps = ROS_ANGLE * 1.7; // convert from degrees to steps

    if (ROS_ANGLE > (0))
    {
        Steer_Position_Steps(P_Steering, ROS_Steps);
    }

    else if (ROS_ANGLE < (0))
    {
        Steer_Position_Steps(P_Steering, ROS_Steps);
    }
    else if (ROS_ANGLE == 0)
    {
        Steer_Position_Steps(P_Steering, ROS_Steps);
    }
}

void RC_Control(int RC_Val)
{

    if (RC_Val > (RC_Steer_Zero + 2))
    {
        int SteeringRC = map(RC_Val, RC_Steer_Zero, RC_Steer_Max, Zero_Angle, Max_Angle);
        Steer_Position(P_Steering, SteeringRC);
    }

    else if (RC_Val < (RC_Steer_Zero - 2))
    {
        int SteeringRC = map(RC_Val, RC_Steer_Min, RC_Steer_Zero, Min_Angle, Zero_Angle);
        Steer_Position(P_Steering, SteeringRC);
    }
    else if (RC_Val == RC_Steer_Zero)
    {
        Steer_Position(P_Steering, Zero_Angle);
    }
}
unsigned long prevMillis = 0;
void Steer_Position_Steps(float &Position, int Step_goal)
{
    // Serial.print("Step_goal:");
    // Serial.println(Step_goal);

    // Serial.printf("Position: %f \n", Position);

    float steps = abs((Position - Step_goal) / 4);
    unsigned short duration = 40;
    if (millis() - prevMillis > duration)
    {
        if (Position > Step_goal)
        {
            Position -= steps;

            Serial.println("left");
            // delay(duration);
        }

        else if (Position < Step_goal)
        {
            Position += steps;

            Serial.println("right");
            // delay(duration);
        }
        else if (Position >= Step_goal || Position <= Step_goal)
        {
            Position = Position;
            GoalState = LOW;
            // test to rptate to the desired position directly but at low speed
        }

        pack_cmd(Steering_Can_ID, Position);
        prevMillis = millis();
    }
}

void RC_Control_Steps(int RC_Val)
{

    // int SteeringRC = map(RC_Val, RC_Steer_Zero, RC_Steer_Max, P_SteeringZero, P_SteeringMin);
    int SteeringRC = map(RC_Val, RC_Steer_Zero, RC_Steer_Max, P_SteeringZero, P_SteeringMax);

    if (RC_Val > (RC_Steer_Zero + 2))
    {
        Steer_Position_Steps(P_Steering, SteeringRC);
    }

    else if (RC_Val < (RC_Steer_Zero - 2))
    {
        // int SteeringRC = map(RC_Val, RC_Steer_Min, RC_Steer_Zero,P_SteeringMax , P_SteeringZero);
        int SteeringRC = map(RC_Val, RC_Steer_Min, RC_Steer_Zero, P_SteeringMin, P_SteeringZero);

        Steer_Position_Steps(P_Steering, SteeringRC);
    }
    else if (RC_Val == RC_Steer_Zero)
    {
        Steer_Position_Steps(P_Steering, P_SteeringZero);
    }
}

void Serial_Control()
{
    char Serial_Data = ' ';
    float Position_step = 1.0;
    int delay_time = 100;

    // int Encoder_Val = Get_Pos(Encoder_Pin);

    if (Serial.available())
    {
        Serial_Data = Serial.read();
        Serial.println("-----------");
        Serial.printf("Serial: %c \n", Serial_Data);
    }

    if (Serial_Data == '1')
        enableMode(Steering_Can_ID);

    else if (Serial_Data == '2')
    {
        Exit(Steering_Can_ID);
        GoalState = LOW;
    }

    else if (Serial_Data == '3')
    {
        // Rotate ClockWise
        Serial.println("Rotate CW");

        if (P_Steering < P_SteeringMax)
            P_Steering += Position_step;
        pack_cmd(Steering_Can_ID, P_Steering);

        Serial.printf("P_Steering: %f \n", P_Steering);
        delay(delay_time);
    }

    else if (Serial_Data == '4')
    {
        // Rotate CounterClockWise
        Serial.println("Rotate CCW");
        if (P_Steering > P_SteeringMin)
            P_Steering -= Position_step;
        pack_cmd(Steering_Can_ID, P_Steering);

        Serial.printf("P_Steering: %f \n", P_Steering);
        delay(delay_time);
    }

    else if (Serial_Data == '0')
    {
        // Go Zero Position
        Serial.println("Zero Position");

        GoalState = HIGH;
    }

    if (GoalState == HIGH)
    {
        Steer_Position(P_Steering, Zero_Angle);
    }
}

int Get_Pos(int Encoder_pin)
{
    float avg = 0;
    for (int i = 0; i < 50; i++)
    {
        avg += analogRead(Encoder_pin);
    }
    avg = avg / 50;
    // Serial.print("")
    // Serial.println(int(avg));
    return avg;
}

void canSniff(const CAN_message_t &msg)
{
    Serial.print("MB ");
    Serial.print(msg.mb);
    Serial.print(" OVERRUN: ");
    Serial.print(msg.flags.overrun);
    Serial.print(" LEN: ");
    Serial.print(msg.len);
    Serial.print(" EXT: ");
    Serial.print(msg.flags.extended);
    Serial.print(" TS: ");
    Serial.print(msg.timestamp);
    Serial.print(" ID: ");
    Serial.print(msg.id, HEX);
    Serial.print(" Buffer: ");
    for (uint8_t i = 0; i < msg.len; i++)
    {
        Serial.print(msg.buf[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}

unsigned int float_to_uint(float x, float x_min, float x_max, int bit)
{ /// converts a float to an unsigned int, given range and number of bits ///

    float span = x_max - x_min;
    float offset = x_min;
    // Serial.println(offset);
    unsigned int pgg = 0;
    if (bit == 12)
    {
        pgg = (x - offset) * 4095 / span;
        return pgg;
    }
    if (bit == 16)
    {
        pgg = (x - offset) * 65535 / span;
        return pgg;
    }
    return 0;
}

void pack_cmd(int ID, float &position)
{
    //  byte buf[8];
    CAN_message_t msg;
    msg.id = ID;

    /// limit data to be within bounds ///

    float p_des = constrain(position, P_SteeringMin, P_SteeringMax);
    float v_des = constrain(v_in, V_MIN, V_MAX);
    float kp = constrain(kp_in, KP_MIN, KP_MAX);
    float kd = constrain(kd_in, KD_MIN, KD_MAX);
    float t_ff = constrain(t_in, T_MIN, T_MAX);
    // convert floats to unsigned ints ///
    unsigned int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
    unsigned int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
    unsigned int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    unsigned int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    unsigned int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
    // pack ints into the can buffer ///

    msg.buf[0] = p_int >> 8;
    msg.buf[1] = p_int & 0xFF;
    msg.buf[2] = v_int >> 4;
    msg.buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    msg.buf[4] = kp_int & 0xFF;
    msg.buf[5] = kd_int >> 4;
    msg.buf[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
    msg.buf[7] = t_int & 0xFF;

    Can3.write(msg);

    delay(10);
}

void enableMode(int ID)
{
    Serial.println("Enter Motor Mode");
    CAN_message_t msg;
    msg.id = ID;
    msg.buf[0] = 0xFF;
    msg.buf[1] = 0xFF;
    msg.buf[2] = 0xFF;
    msg.buf[3] = 0xFF;
    msg.buf[4] = 0xFF;
    msg.buf[5] = 0xFF;
    msg.buf[6] = 0xFF;
    msg.buf[7] = 0xFC;
    Can3.write(msg);
}

void Exit(int ID)
{
    Serial.println("Exit");
    CAN_message_t msg;
    msg.id = ID;
    msg.buf[0] = 0xFF;
    msg.buf[1] = 0xFF;
    msg.buf[2] = 0xFF;
    msg.buf[3] = 0xFF;
    msg.buf[4] = 0xFF;
    msg.buf[5] = 0xFF;
    msg.buf[6] = 0xFF;
    msg.buf[7] = 0xFD;
    Can3.write(msg);
}

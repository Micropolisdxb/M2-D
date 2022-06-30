#include <Brake.h>
#include <Arduino.h>

void Brake_init()
{
    pinMode(Brake_Dir, OUTPUT);
    pinMode(Brake_PWM, OUTPUT);

    Serial.println("Braking Init");
    digitalWrite(Brake_Dir, HIGH);
    digitalWrite(Brake_PWM, HIGH);
    delay(1000);
    digitalWrite(Brake_PWM, LOW);
}
int previous_throttle = 0;
int Release_State = 0;
int Brake_State = 0;

unsigned long prevMillis_brake = 0;
int STOP_CYTRON = LOW;
void Brake_Control_ROS(int Throttle_Value)
{

    if (Throttle_Value > 0)
    {
        STOP_CYTRON = LOW;

        // Throttle_Value = map(Throttle_Value, Throttle_Zero, Throttle_Max, 0, 100);
        //  Serial.printf("Throttle_Value: %d " , Throttle_Value);
        //  Serial.printf("  previous_throttle: %d \n " , previous_throttle);

        if ((Throttle_Value - previous_throttle > 1) && Release_State == LOW)
        {
            digitalWrite(Brake_Dir, LOW);
            analogWrite(Brake_PWM, 125);

            Serial.printf("Forward  Releasing %d  \n", Throttle_Value);

            Brake_State = LOW;
            Release_State = HIGH;

            prevMillis_brake = millis();
            previous_throttle = Throttle_Value;
        }

        else if ((Throttle_Value - previous_throttle < -1) && Throttle_Value < 15 && (Brake_State == LOW))
        {
            digitalWrite(Brake_Dir, HIGH);
            analogWrite(Brake_PWM, 125);

            Serial.printf("Forward Braking %d  \n", Throttle_Value);

            Brake_State = HIGH;
            Release_State = LOW;
            prevMillis_brake = millis();
            previous_throttle = Throttle_Value;
        }
    }

    else if (Throttle_Value < 0)
    {
        STOP_CYTRON = LOW;
        Throttle_Value = abs(Throttle_Value);

        // Throttle_Value = map(Throttle_Value, Throttle_Min, Throttle_Zero, 100, 0);
        // Serial.printf("Throttle_Value: %d " , Throttle_Value);
        // Serial.printf("  previous_throttle: %d \n " , previous_throttle);

        if ((Throttle_Value - previous_throttle > 1))
        {
            digitalWrite(Brake_Dir, LOW);
            analogWrite(Brake_PWM, 125);

            Serial.printf("Reverse Releasing %d  \n", Throttle_Value);

            Brake_State = LOW;
            Release_State = HIGH;
            prevMillis_brake = millis();
            previous_throttle = Throttle_Value;
        }

        else if ((Throttle_Value - previous_throttle < -1) && Throttle_Value < 15 && (Brake_State == LOW))
        {
            digitalWrite(Brake_Dir, HIGH);
            analogWrite(Brake_PWM, 125);

            Serial.printf("Reverse Braking %d  \n", Throttle_Value);

            Brake_State = HIGH;
            Release_State = LOW;
            prevMillis_brake = millis();
            previous_throttle = Throttle_Value;
        }
    }

    else if (Throttle_Value == 0 && (Brake_State == LOW))
    {
        if (!STOP_CYTRON)
        {
            digitalWrite(Brake_Dir, HIGH);
            analogWrite(Brake_PWM, 125);
        }
        previous_throttle = 0;
        prevMillis_brake = millis();
        Brake_State = HIGH;
    }

    if ((millis() - prevMillis_brake > 1000) && ((Brake_State == HIGH) || Release_State == HIGH))
    {
        Serial.println("--------------Stop Brake--------------");
        analogWrite(Brake_PWM, 0);
        Brake_State = LOW;
        Release_State = LOW;
        STOP_CYTRON = HIGH;

    }
}

void Brake_Control_RC(int Throttle_Value)
{

    if (Throttle_Value > 1003)
    {
        STOP_CYTRON = LOW;

        Throttle_Value = map(Throttle_Value, RC_Throttle_Zero, RC_Throttle_Max, 0, 100);
        // Serial.printf("Throttle_Value: %d " , Throttle_Value);
        // Serial.printf("  previous_throttle: %d \n " , previous_throttle);

        if ((Throttle_Value - previous_throttle > 1) && Release_State == LOW)
        {
            digitalWrite(Brake_Dir, LOW);
            analogWrite(Brake_PWM, 125);

            Serial.printf("Forward  Releasing %d  \n", Throttle_Value);

            Brake_State = LOW;
            Release_State = HIGH;

            prevMillis_brake = millis();
            previous_throttle = Throttle_Value;
        }

        else if ((Throttle_Value - previous_throttle < -1) && Throttle_Value < 15 && (Brake_State == LOW))
        {
            digitalWrite(Brake_Dir, HIGH);
            analogWrite(Brake_PWM, 125);

            Serial.printf("Forward Braking %d  \n", Throttle_Value);


            Brake_State = HIGH;
            Release_State = LOW;
            prevMillis_brake = millis();
            previous_throttle = Throttle_Value;
        }
    }

    else if (Throttle_Value < 1000)
    {
        STOP_CYTRON = LOW;

        Throttle_Value = map(Throttle_Value, RC_Throttle_Min, RC_Throttle_Zero, 100, 0);
        // Serial.printf("Throttle_Value: %d " , Throttle_Value);
        // Serial.printf("  previous_throttle: %d \n " , previous_throttle);

        if ((Throttle_Value - previous_throttle > 1))
        {
            digitalWrite(Brake_Dir, LOW);
            analogWrite(Brake_PWM, 125);

            Serial.printf("Reverse Releasing %d  \n", Throttle_Value);

            Brake_State = LOW;
            Release_State = HIGH;
            prevMillis_brake = millis();
            previous_throttle = Throttle_Value;
        }

        else if ((Throttle_Value - previous_throttle < -1) && Throttle_Value < 15 && (Brake_State == LOW))
        {
            digitalWrite(Brake_Dir, HIGH);
            analogWrite(Brake_PWM, 125);

            Serial.printf("Reverse Braking %d  \n", Throttle_Value);

            Brake_State = HIGH;
            Release_State = LOW;
            prevMillis_brake = millis();
            previous_throttle = Throttle_Value;
        }
    }

    else if (Throttle_Value == 1002 && (Brake_State == LOW))
    {
                if (!STOP_CYTRON)
        {
            digitalWrite(Brake_Dir, HIGH);
            analogWrite(Brake_PWM, 125);
        }
        previous_throttle = 0;
        prevMillis_brake = millis();
        Brake_State = HIGH;
    }

    if ((millis() - prevMillis_brake > 1000) && ((Brake_State == HIGH) || Release_State == HIGH))
    {
        Serial.println("--------------Stop Brake--------------");
        analogWrite(Brake_PWM, 0);
        Brake_State = LOW;
        Release_State = LOW;
    }
}

void Brake_Control_Serial()
{
    char Serial_Data = ' ';

    if (Serial.available())
    {
        Serial_Data = Serial.read();
        Serial.println("-----------");
        Serial.printf("Serial: %c \n", Serial_Data);
    }

    if (Serial_Data == '1')
    {
        Serial.println("Direction HIGH brake");
        digitalWrite(Brake_Dir, HIGH);
        analogWrite(Brake_PWM, 125);
        prevMillis_brake = millis();
    }

    else if (Serial_Data == '2')
    {
        Serial.println("Direction LOW release");
        digitalWrite(Brake_Dir, LOW);
        analogWrite(Brake_PWM, 125);
        prevMillis_brake = millis();
    }
    else if (Serial_Data == '3')
    {
        Serial.println("Direction LOW");
        digitalWrite(Brake_Dir, LOW);
        analogWrite(Brake_PWM, 0);
    }
    if (millis() - prevMillis_brake > 1000)
    {
        analogWrite(Brake_PWM, 0);
    }
}
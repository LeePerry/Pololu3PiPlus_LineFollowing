#include "Motors.h"

#include <Arduino.h>

#define RIGHT_MOTOR_PWM        9
#define LEFT_MOTOR_PWM        10
#define RIGHT_MOTOR_DIRECTION 15
#define LEFT_MOTOR_DIRECTION  16

#define FORWARD LOW
#define REVERSE HIGH

namespace Hardware
{
    void Motors::Initialise()
    {
        pinMode(RIGHT_MOTOR_PWM,       OUTPUT);
        pinMode(LEFT_MOTOR_PWM,        OUTPUT);
        pinMode(RIGHT_MOTOR_DIRECTION, OUTPUT);
        pinMode(LEFT_MOTOR_DIRECTION,  OUTPUT);

        SetSpeeds(0, 0);
    }

    void Motors::SetSpeeds(const Speed left, const Speed right)
    {
        SetIndividualSpeed(LEFT_MOTOR_PWM,  LEFT_MOTOR_DIRECTION,  left);
        SetIndividualSpeed(RIGHT_MOTOR_PWM, RIGHT_MOTOR_DIRECTION, right);
    }

    void Motors::SetIndividualSpeed(const Pin pwm, const Pin direction, const Speed speed)
    {
        if (speed > 0)
        {
            digitalWrite(direction, FORWARD);
            analogWrite(pwm, speed);
        }
        else
        {
            digitalWrite(direction, REVERSE);
            analogWrite(pwm, -speed);
        }
    }
}
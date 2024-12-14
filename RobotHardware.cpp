#include "RobotHardware.h"

#include "Motors.h"
#include "Sensors.h"
#include "Kinematics.h"

#include <Arduino.h>

#define YELLOW_LED 13

namespace Hardware
{
    Motors motors;
    Sensors sensors;
    Kinematics kinematics;

    void Initialise()
    {
        motors.Initialise();
        sensors.Initialise();
        kinematics.Initialise();

        pinMode(YELLOW_LED, OUTPUT);
    }

    Perception Perceive()
    {
        Perception perception;
        perception.now  = millis();
        perception.line = sensors.ReadBlackLine();
        perception.pose = kinematics.GlobalPose();
        return perception;
    }

    void Act(const Decision& decision)
    {
        motors.SetSpeeds(decision.leftWheel, decision.rightWheel);
        digitalWrite(YELLOW_LED, decision.yellowLed ? HIGH : LOW);
    }
}
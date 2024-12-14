#pragma once

#include "Speed.h"

#define MIN_WHEEL_SPEED -80
#define MAX_WHEEL_SPEED 80

struct Decision
{
    Hardware::Speed leftWheel = 0;
    Hardware::Speed rightWheel = 0;
    bool yellowLed = false;
};

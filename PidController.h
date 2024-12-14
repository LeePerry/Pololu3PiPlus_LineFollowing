#pragma once

#include "Decision.h"
#include "Perception.h"

namespace Controllers
{
    void Pid(const int16_t error, Hardware::Speed& leftWheel, Hardware::Speed& rightWheel);
}
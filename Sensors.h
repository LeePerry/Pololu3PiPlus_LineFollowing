#pragma once

#include "Line.h"

namespace Hardware
{
    class Sensors
    {
        public:
            void Initialise();
            Line ReadBlackLine();
    };
}
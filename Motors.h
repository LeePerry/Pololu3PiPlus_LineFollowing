#pragma once

#include "Pin.h"
#include "Speed.h"

namespace Hardware
{
    class Motors
    {
        public:
            void Initialise();
            void SetSpeeds(const Speed left, const Speed right);

        private:
            void SetIndividualSpeed(const Pin pwm, const Pin direction, const Speed speed);
    };
}
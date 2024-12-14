#include "PidController.h"

#include <Arduino.h>

#define PROPORTIONAL_DENOMINATOR 10
#define DIFFERENTIAL_DENOMINATOR 4

// max = max_int16_t, min = std::abs(max_error)
#define FORWARD_SPEED_DENOMINATOR 2000

namespace Controllers
{
    void Pid(const int16_t error, Hardware::Speed& leftWheel, Hardware::Speed& rightWheel)
    {
        static int16_t previousError = 0;
        
        // Doesn't have much impact, but theoretically it slows forward motion while we're turning
        const auto forwardControl = (MAX_WHEEL_SPEED * (FORWARD_SPEED_DENOMINATOR - abs(error))) / FORWARD_SPEED_DENOMINATOR;
        
        // This is doing the bulk of the work, controlling the turning speeds and minimising the error
        const auto turningControl = (error / PROPORTIONAL_DENOMINATOR) +
                                    ((error - previousError) / DIFFERENTIAL_DENOMINATOR);

        previousError = error;

        leftWheel  = forwardControl + turningControl;
        rightWheel = forwardControl - turningControl;
    }
}
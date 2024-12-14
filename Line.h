#pragma once

#include <stdint.h>

struct Line
{
    bool detected = false;
    int16_t positionEstimate = 0;
};
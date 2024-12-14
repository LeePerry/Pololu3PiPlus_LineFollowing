#pragma once

#include "Pose.h"
#include "Line.h"

using Milliseconds = unsigned long;

struct Perception
{
    Milliseconds now = 0;
    Line line;
    Pose pose;
};

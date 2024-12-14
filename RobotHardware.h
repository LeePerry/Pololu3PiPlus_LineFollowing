#pragma once

#include "Perception.h"
#include "Decision.h"

namespace Hardware
{
    void Initialise();
    Perception Perceive();
    void Act(const Decision& decision);
}

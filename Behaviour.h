#pragma once

#include "Decision.h"
#include "Perception.h"

namespace Behaviour
{
    Decision Decide(const Perception& perception);
}
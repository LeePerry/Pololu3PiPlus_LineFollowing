#include "Behaviour.h"

#include "LineFollowing.h"

namespace Behaviour
{
    Decision Decide(const Perception& perception)
    {
        static LineFollowingStateMachine fsm;
        return fsm.Update(perception);
    }
}
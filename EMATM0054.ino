#include "Behaviour.h"
#include "RobotHardware.h"

#define UPDATE_INTERVAL_MS 5

void setup()
{
    Hardware::Initialise();
}

void loop()
{
    /* Conceptually the loop is best understood as Perception -> Decision -> Action.
     *
     * However we actually schedule each update to start with Action, ensuring it has
     * the most uniform interval and predictable behaviour.
     *
     * Perception and Decision are then given the remainder of the update interval,
     * in order to prepare data for the following update.
     */

    static Perception perception;
    static Decision decision;
    static Milliseconds nextUpdateScheduled = 0;
    const auto now = millis();

    if (nextUpdateScheduled < now)
    {
        nextUpdateScheduled = now + UPDATE_INTERVAL_MS;
        Hardware::Act(decision);
        perception = Hardware::Perceive();
        decision = Behaviour::Decide(perception);
    }
}
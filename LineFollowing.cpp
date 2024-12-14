#include "LineFollowing.h"

#include <Arduino.h>

#include "PidController.h"

#define COURSE_END_DISTANCE_MM 500
#define ARRIVAL_THREHSOLD 70

#define TRANSITION_TO_STATE(state) \
    Transition(&LineFollowingStateMachine::state, perception.now); \
    //Serial.println(#state);

namespace
{
    long DistanceToStartMm(const Perception& perception)
    {
        return sqrt(sq(perception.pose.x) + sq(perception.pose.y));
    }

    double DirectionToStart(const Perception& perception)
    {
        const auto trajectoryStartX = -perception.pose.x;
        const auto trajectoryStartY = -perception.pose.y;
        const auto distanceToStart = DistanceToStartMm(perception);
        const auto currentTrajectoryX = distanceToStart * cos(perception.pose.theta);
        const auto currentTrajectoryY = distanceToStart * sin(perception.pose.theta);
        return atan2((trajectoryStartY * currentTrajectoryX) - (trajectoryStartX * currentTrajectoryY),
                     (trajectoryStartY * currentTrajectoryY) + (trajectoryStartX * currentTrajectoryX));
    }
}

namespace Behaviour
{
    LineFollowingStateMachine::LineFollowingStateMachine() :
        m_state(&LineFollowingStateMachine::JoinLine),
        m_lastTransition(0)
    {}

    Decision LineFollowingStateMachine::Update(const Perception& perception)
    {
        Decision decision;
        decision.yellowLed = perception.line.detected;
        (this->*m_state)(perception, decision);
        return decision;
    }

    void LineFollowingStateMachine::Transition(const State state, const Milliseconds now)
    {
        m_state = state;
        m_lastTransition = now;
    }

    void LineFollowingStateMachine::JoinLine(const Perception& perception, Decision& decision)
    {
        decision.leftWheel  = MAX_WHEEL_SPEED / 2;
        decision.rightWheel = MAX_WHEEL_SPEED / 2;

        if (perception.line.detected)
        {
            TRANSITION_TO_STATE(CentraliseLine);
        }
    }

    void LineFollowingStateMachine::CentraliseLine(const Perception& perception, Decision& decision)
    {
        if ((m_lastTransition + 200) > perception.now)
        {
            decision.leftWheel  = MAX_WHEEL_SPEED / 2;
            decision.rightWheel = MAX_WHEEL_SPEED / 2;
        }
        else
        {
            decision.leftWheel  = 0;
            decision.rightWheel = 0;
            TRANSITION_TO_STATE(Rotate90);
        }
    }

    void LineFollowingStateMachine::Rotate90(const Perception& perception, Decision& decision)
    {
        if ((m_lastTransition + 350) > perception.now)
        {
            decision.leftWheel  = 40;
            decision.rightWheel = -40;
        }
        else
        {
            decision.leftWheel  = 0;
            decision.rightWheel = 0;
            TRANSITION_TO_STATE(FollowLine);
        }
    }

    void LineFollowingStateMachine::FollowLine(const Perception& perception, Decision& decision)
    {
        if (perception.line.detected)
        {
            Controllers::Pid(perception.line.positionEstimate, decision.leftWheel, decision.rightWheel);
        }
        else
        {
            TRANSITION_TO_STATE(LostLine);
        }
    }
    
    void LineFollowingStateMachine::LostLine(const Perception& perception, Decision& decision)
    {
        if (perception.line.detected)
        {
            // line found - return to line following
            Controllers::Pid(perception.line.positionEstimate, decision.leftWheel, decision.rightWheel);
            TRANSITION_TO_STATE(FollowLine);
        }
        else if ((m_lastTransition + 300) > perception.now)
        {
            // potentially a gap - pretend the goal is directly ahead
            Controllers::Pid(0, decision.leftWheel, decision.rightWheel);
        }
        else
        {
            // confirmed lost - let's turn around and retrace our steps
            decision.leftWheel  = 0;
            decision.rightWheel = 0;

            if (DistanceToStartMm(perception) > COURSE_END_DISTANCE_MM)
            {
                TRANSITION_TO_STATE(Pause);
            }
            else
            {
                TRANSITION_TO_STATE(Rotate180);
            }
        }
    }

    void LineFollowingStateMachine::Rotate180(const Perception& perception, Decision& decision)
    {
        if ((m_lastTransition + 700) > perception.now)
        {
            decision.leftWheel  = -40;
            decision.rightWheel = 40;
        }
        else
        {
            decision.leftWheel  = 0;
            decision.rightWheel = 0;
            TRANSITION_TO_STATE(ReturnToLine);
        }
    }

    void LineFollowingStateMachine::ReturnToLine(const Perception& perception, Decision& decision)
    {
        decision.leftWheel  = MAX_WHEEL_SPEED;
        decision.rightWheel = MAX_WHEEL_SPEED;

        if (perception.line.detected)
        {
            TRANSITION_TO_STATE(FollowLine);
        }
    }

    void LineFollowingStateMachine::Pause(const Perception& perception, Decision& decision)
    {
        decision.leftWheel  = 0;
        decision.rightWheel = 0;

        if ((m_lastTransition + 2000) < perception.now)
        {
            TRANSITION_TO_STATE(ReturnToStart);
        }
    }

    void LineFollowingStateMachine::ReturnToStart(const Perception& perception, Decision& decision)
    {
        if (DistanceToStartMm(perception) < ARRIVAL_THREHSOLD)
        {
            // Yay! We're home!!!
            decision.leftWheel  = 0;
            decision.rightWheel = 0;
        }
        else
        {
            // Move towards home
            Controllers::Pid(DirectionToStart(perception) * 1000, decision.leftWheel, decision.rightWheel);
        }
    }
}
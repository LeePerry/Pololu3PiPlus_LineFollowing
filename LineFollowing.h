#pragma once

#include "Decision.h"
#include "Perception.h"

namespace Behaviour
{
    class LineFollowingStateMachine
    {
        public:
            LineFollowingStateMachine();
            Decision Update(const Perception& perception);

        private:
            using State = void (LineFollowingStateMachine::*)(const Perception&, Decision&);

            State m_state;
            Milliseconds m_lastTransition;

            void Transition(const State state, const Milliseconds now);

            // States
            void JoinLine       (const Perception& perception, Decision& decision);
            void CentraliseLine (const Perception& perception, Decision& decision);
            void Rotate90       (const Perception& perception, Decision& decision);
            void FollowLine     (const Perception& perception, Decision& decision);
            void LostLine       (const Perception& perception, Decision& decision);
            void Rotate180      (const Perception& perception, Decision& decision);
            void ReturnToLine   (const Perception& perception, Decision& decision);
            void Pause          (const Perception& perception, Decision& decision);
            void ReturnToStart  (const Perception& perception, Decision& decision);
    };
}
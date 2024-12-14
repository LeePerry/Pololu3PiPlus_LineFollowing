#pragma once

#include "Pose.h"

namespace Hardware
{
    class Kinematics
    {
        public:
            Kinematics();
            void Initialise();
            Pose GlobalPose();

        private:
            long m_previousLeftCount;
            long m_previousRightCount;

            double m_x;
            double m_y;
            double m_theta;
    };
}
#include "Kinematics.h"

#include "Encoders.h"

#include <Arduino.h>

#define WHEEL_RADIUS_MM 16
#define COUNTS_PER_ROTATION 358.3
#define WHEEL_BASE_MM 84.0

namespace Hardware
{
    Kinematics::Kinematics() :
        m_previousLeftCount(0),
        m_previousRightCount(0),
        m_x(0.0),
        m_y(0.0),
        m_theta(0.0)
    {}

    void Kinematics::Initialise()
    {
        SetupRightEncoder();
        SetupLeftEncoder();
    }

    Pose Kinematics::GlobalPose()
    {
        noInterrupts();
        const auto leftCount  = LeftEncoderCount();
        const auto rightCount = RightEncoderCount();
        interrupts();

        const auto leftDelta  = leftCount  - m_previousLeftCount;
        const auto rightDelta = rightCount - m_previousRightCount;

        if (leftDelta != 0 || rightDelta != 0)
        {
            // we need to recalculate a new global pose
            m_previousLeftCount  = leftCount;
            m_previousRightCount = rightCount;

            const auto leftDistance  = (leftDelta  * TWO_PI * WHEEL_RADIUS_MM) / COUNTS_PER_ROTATION;
            const auto rightDistance = (rightDelta * TWO_PI * WHEEL_RADIUS_MM) / COUNTS_PER_ROTATION;
            const auto distanceDelta = (leftDistance + rightDistance) / 2.0;
            const auto rotationDelta = (leftDistance - rightDistance) / WHEEL_BASE_MM;

            if (rotationDelta == 0.0)
            {
                // linear movement
                m_x += distanceDelta * cos(m_theta);
                m_y += distanceDelta * sin(m_theta);
            }
            else if (distanceDelta == 0.0)
            {
                // spot rotation
                m_theta += rotationDelta;
                m_theta = fmod(m_theta, TWO_PI);
            }
            else
            {
                // arc movement
                // first calc x and y delta in robot local co-ordinates
                const auto rotationRadius = distanceDelta / rotationDelta;
                const auto localXDelta = rotationRadius * sin(rotationDelta);
                const auto localYDelta = rotationDelta < 0.0 ? 
                                        -rotationRadius + (rotationRadius * cos(rotationDelta)) : 
                                          rotationRadius - (rotationRadius * cos(rotationDelta));

                // then convert back to global co-ordinates
                m_x += (localXDelta * cos(m_theta)) + (localYDelta * sin(m_theta));
                m_y += (localXDelta * sin(m_theta)) + (localYDelta * cos(m_theta));
                m_theta += rotationDelta;
                m_theta = fmod(m_theta, TWO_PI);
            }
        }

        // we store and calulate in doubles for maximum accuracy
        // only convert back to integers when publishing global pose
        Pose pose;
        pose.x = m_x;
        pose.y = m_y;
        pose.theta = m_theta;
        return pose;
    }
}
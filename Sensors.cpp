#include "Sensors.h"

#include "Pin.h"

#include <Arduino.h>

#define NUMBER_OF_SENSORS    5

#define FAR_LEFT_SENSOR_PIN  12
#define LEFT_SENSOR_PIN      A0
#define CENTRE_SENSOR_PIN    A2
#define RIGHT_SENSOR_PIN     A3
#define FAR_RIGHT_SENSOR_PIN A4

#define EMIT_PIN 11

#define TIMEOUT 2500
#define INITIAL_VALUE TIMEOUT

#define LINE_ESTIMATE_PRECISION 1000

#define WHITE_THRESHOLD 900  // anything under this is dicounted as noise
#define BLACK_THRESHOLD 1500 // anything over this is considered a black line detection

namespace 
{
    using Microseconds = unsigned long;
    using Readings = Microseconds[NUMBER_OF_SENSORS];

    Hardware::Pin ALL_SENSORS[NUMBER_OF_SENSORS] = {FAR_LEFT_SENSOR_PIN, 
                                                    LEFT_SENSOR_PIN, 
                                                    CENTRE_SENSOR_PIN, 
                                                    RIGHT_SENSOR_PIN, 
                                                    FAR_RIGHT_SENSOR_PIN};

    void SetAllPinModes(const int mode)
    {
        for (const auto pin : ALL_SENSORS)
        {
            pinMode(pin, mode);
        }
    }

    void DigitalWriteAll(const int value)
    {
        for (const auto pin : ALL_SENSORS)
        {
            digitalWrite(pin, value);
        }
    }

    void ReadSensors(Readings& readings)
    {
        // initialise sensor read times
        const auto started = micros();
        for (int i = 0; i < NUMBER_OF_SENSORS; ++i)
        {
            readings[i] = INITIAL_VALUE;
        }

        // charge capacitors
        SetAllPinModes(OUTPUT);
        DigitalWriteAll(HIGH);
        delayMicroseconds(10);
        SetAllPinModes(INPUT);

        // read sensors
        auto remaining = NUMBER_OF_SENSORS;
        while (remaining > 0)
        {
            Microseconds elapsed_time = micros() - started;

            if (elapsed_time > TIMEOUT)
            {
                break;
            }

            for (int i = 0; i < NUMBER_OF_SENSORS; ++i)
            {
                if ((readings[i] == INITIAL_VALUE) && (digitalRead(ALL_SENSORS[i]) == LOW))
                {
                    readings[i] = elapsed_time;
                    --remaining;
                }
            }
        }
    }

    bool IsLineDetected(const Readings& readings)
    {
        for (int i = 0; i < NUMBER_OF_SENSORS; ++i)
        {
            if (readings[i] > BLACK_THRESHOLD)
            {
                return true;
            }
        }
        return false;
    }

    int16_t MostLikelyLinePosition(const Readings& readings)
    {
        Microseconds average = 0;
        Microseconds total = 0;
        
        for (int i = 0; i < NUMBER_OF_SENSORS; ++i)
        {
            const auto value = readings[i] < WHITE_THRESHOLD ? 0 : readings[i] - WHITE_THRESHOLD;
            average += value * i * LINE_ESTIMATE_PRECISION;
            total += value;
        }
        
        return static_cast<int16_t>(average / total) - 
            static_cast<int16_t>((NUMBER_OF_SENSORS - 1) * LINE_ESTIMATE_PRECISION / 2);
    }
}

namespace Hardware
{
    void Sensors::Initialise()
    {
        SetAllPinModes(INPUT);
        pinMode(EMIT_PIN, OUTPUT);
        digitalWrite(EMIT_PIN, HIGH);
    }

    Line Sensors::ReadBlackLine()
    {
        Readings readings;
        ReadSensors(readings);
        
        Line line;
        line.detected = IsLineDetected(readings);
        if (line.detected)
        {
            line.positionEstimate = MostLikelyLinePosition(readings);
        }
        return line;
    }
}

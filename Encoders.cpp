#include "Encoders.h"

#include <Arduino.h>

#define ENCODER_0_A_PIN 7
#define ENCODER_0_B_PIN 23
#define ENCODER_1_A_PIN 26

namespace
{
    volatile byte stateEncoderLeft;
    volatile byte stateEncoderRight;

    volatile long countEncoderLeft;
    volatile long countEncoderRight;
}

namespace Hardware
{
    void SetupLeftEncoder()
    {
        DDRE = DDRE & ~(1 << DDE6);
        PORTE = PORTE | (1 << PORTE2);

        pinMode(ENCODER_1_A_PIN, INPUT);
        digitalWrite(ENCODER_1_A_PIN, HIGH);

        countEncoderLeft = 0;
        stateEncoderLeft = 0;

        boolean encoderLeftPinB = PINE & (1 << PINE2);
        boolean encoderLeftPinA = digitalRead(ENCODER_1_A_PIN);
        encoderLeftPinA = encoderLeftPinA ^ encoderLeftPinB;
        stateEncoderLeft = stateEncoderLeft | (encoderLeftPinB << 1);
        stateEncoderLeft = stateEncoderLeft | (encoderLeftPinA << 0);

        PCICR = PCICR & ~(1 << PCIE0);
        PCMSK0 |= (1 << PCINT4);
        PCIFR |= (1 << PCIF0);
        PCICR |= (1 << PCIE0);
    }

    void SetupRightEncoder()
    {
        pinMode(ENCODER_0_A_PIN, INPUT);
        pinMode(ENCODER_0_B_PIN, INPUT);

        countEncoderRight = 0;
        stateEncoderRight = 0;

        boolean encoderRightPinA = digitalRead(ENCODER_0_A_PIN);
        boolean encoderRightPinB = digitalRead(ENCODER_0_B_PIN);
        encoderRightPinA = encoderRightPinA ^ encoderRightPinB;
        stateEncoderRight = stateEncoderRight | (encoderRightPinB << 1);
        stateEncoderRight = stateEncoderRight | (encoderRightPinA << 0);

        EIMSK = EIMSK & ~(1 << INT6);
        EICRB |= ( 1 << ISC60 );
        EIFR |= ( 1 << INTF6 );
        EIMSK |= ( 1 << INT6 );
    }

    long LeftEncoderCount()
    {
        return countEncoderLeft;
    }

    long RightEncoderCount()
    {
        return countEncoderRight;
    }
}

ISR(PCINT0_vect)
{
    using namespace Hardware;

    boolean encoderLeftPinB = PINE & (1 << PINE2);
    boolean encoderLeftPinA = digitalRead(ENCODER_1_A_PIN);
    encoderLeftPinA = encoderLeftPinA ^ encoderLeftPinB;
    stateEncoderLeft = stateEncoderLeft | (encoderLeftPinB << 3);
    stateEncoderLeft = stateEncoderLeft | (encoderLeftPinA << 2);

    if (stateEncoderLeft == 1 || stateEncoderLeft == 7 || stateEncoderLeft == 8 || stateEncoderLeft == 14)
    {
        countEncoderLeft -= 1;
    }
    else if (stateEncoderLeft == 2 || stateEncoderLeft == 4 || stateEncoderLeft == 11 || stateEncoderLeft == 13)
    {
        countEncoderLeft += 1;
    }

    stateEncoderLeft = stateEncoderLeft >> 2;
}

ISR(INT6_vect)
{
    using namespace Hardware;

    boolean EncoderRightPinB = digitalRead(ENCODER_0_B_PIN);
    boolean EncoderRightPinA = digitalRead(ENCODER_0_A_PIN);
    EncoderRightPinA = EncoderRightPinA ^ EncoderRightPinB;
    stateEncoderRight = stateEncoderRight | (EncoderRightPinB << 3);
    stateEncoderRight = stateEncoderRight | (EncoderRightPinA << 2);

    if (stateEncoderRight == 1 || stateEncoderRight == 7 || stateEncoderRight == 8 || stateEncoderRight == 14)
    {
        countEncoderRight -= 1;
    }
    else if (stateEncoderRight == 2 || stateEncoderRight == 4 || stateEncoderRight == 11 || stateEncoderRight == 13)
    {
        countEncoderRight += 1;
    }

    stateEncoderRight = stateEncoderRight >> 2;
}

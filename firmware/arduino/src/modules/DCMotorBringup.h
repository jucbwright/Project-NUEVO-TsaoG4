#ifndef DC_MOTOR_BRINGUP_H
#define DC_MOTOR_BRINGUP_H

#include <Arduino.h>

#include "../config.h"
#include "../drivers/DCMotor.h"
#include "EncoderCounter.h"
#include "VelocityEstimator.h"

class DCMotorBringup {
public:
    static uint16_t countsPerRev();

    static void initAll(DCMotor *motors,
                        IEncoderCounter &encoder1, EdgeTimeVelocityEstimator &velocity1,
                        IEncoderCounter &encoder2, EdgeTimeVelocityEstimator &velocity2,
                        IEncoderCounter &encoder3, EdgeTimeVelocityEstimator &velocity3,
                        IEncoderCounter &encoder4, EdgeTimeVelocityEstimator &velocity4);

private:
    static void initOne(DCMotor &motor,
                        uint8_t motorId,
                        IEncoderCounter &encoder,
                        EdgeTimeVelocityEstimator &velocityEstimator,
                        uint8_t encA,
                        uint8_t encB,
                        bool encoderInvert,
                        bool motorInvert,
                        uint8_t pinEN,
                        uint8_t pinIN1,
                        uint8_t pinIN2);
};

#endif // DC_MOTOR_BRINGUP_H

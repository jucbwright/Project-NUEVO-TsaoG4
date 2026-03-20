#include "DCMotorBringup.h"

#include "../config.h"
#include "../pins.h"

uint16_t DCMotorBringup::countsPerRev() {
#if ENCODER_1_MODE == ENCODER_4X
    return (uint16_t)(ENCODER_PPR * 4U);
#else
    return (uint16_t)(ENCODER_PPR * 2U);
#endif
}

void DCMotorBringup::initAll(DCMotor *motors,
                             IEncoderCounter &encoder1, EdgeTimeVelocityEstimator &velocity1,
                             IEncoderCounter &encoder2, EdgeTimeVelocityEstimator &velocity2,
                             IEncoderCounter &encoder3, EdgeTimeVelocityEstimator &velocity3,
                             IEncoderCounter &encoder4, EdgeTimeVelocityEstimator &velocity4) {
    initOne(motors[0], 0, encoder1, velocity1,
            PIN_M1_ENC_A, PIN_M1_ENC_B, ENCODER_1_DIR_INVERTED, DC_MOTOR_1_DIR_INVERTED,
            PIN_M1_EN, PIN_M1_IN1, PIN_M1_IN2);
    initOne(motors[1], 1, encoder2, velocity2,
            PIN_M2_ENC_A, PIN_M2_ENC_B, ENCODER_2_DIR_INVERTED, DC_MOTOR_2_DIR_INVERTED,
            PIN_M2_EN, PIN_M2_IN1, PIN_M2_IN2);
    initOne(motors[2], 2, encoder3, velocity3,
            PIN_M3_ENC_A, PIN_M3_ENC_B, ENCODER_3_DIR_INVERTED, DC_MOTOR_3_DIR_INVERTED,
            PIN_M3_EN, PIN_M3_IN1, PIN_M3_IN2);
    initOne(motors[3], 3, encoder4, velocity4,
            PIN_M4_ENC_A, PIN_M4_ENC_B, ENCODER_4_DIR_INVERTED, DC_MOTOR_4_DIR_INVERTED,
            PIN_M4_EN, PIN_M4_IN1, PIN_M4_IN2);
}

void DCMotorBringup::initOne(DCMotor &motor,
                             uint8_t motorId,
                             IEncoderCounter &encoder,
                             EdgeTimeVelocityEstimator &velocityEstimator,
                             uint8_t encA,
                             uint8_t encB,
                             bool encoderInvert,
                             bool motorInvert,
                             uint8_t pinEN,
                             uint8_t pinIN1,
                             uint8_t pinIN2) {
    uint16_t cpr = countsPerRev();

    encoder.init(encA, encB, encoderInvert);
    velocityEstimator.init(cpr);
    velocityEstimator.setFilterSize(VELOCITY_FILTER_SIZE);
    velocityEstimator.setZeroTimeout(VELOCITY_ZERO_TIMEOUT);

    motor.init(motorId, &encoder, &velocityEstimator, motorInvert);
    motor.setPins(pinEN, pinIN1, pinIN2);
    motor.setPositionPID(DEFAULT_POS_KP, DEFAULT_POS_KI, DEFAULT_POS_KD);
    motor.setVelocityPID(DEFAULT_VEL_KP, DEFAULT_VEL_KI, DEFAULT_VEL_KD);
}

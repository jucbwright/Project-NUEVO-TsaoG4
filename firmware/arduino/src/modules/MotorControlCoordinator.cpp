#include "MotorControlCoordinator.h"

#include <util/atomic.h>

#include "../drivers/DCMotor.h"

volatile uint32_t MotorControlCoordinator::roundCount_ = 0;
volatile uint32_t MotorControlCoordinator::requestedRound_ = 0;
volatile uint32_t MotorControlCoordinator::computedRound_ = 0;
volatile uint32_t MotorControlCoordinator::appliedRound_ = 0;
volatile uint8_t MotorControlCoordinator::currentSlot_ = 0;
volatile uint8_t MotorControlCoordinator::isrSlot_ = 0;
volatile uint32_t MotorControlCoordinator::roundStartUs_ = 0;
volatile uint8_t MotorControlCoordinator::computeSeq_ = 0;
volatile uint8_t MotorControlCoordinator::preparedSeq_ = 0;
volatile uint8_t MotorControlCoordinator::appliedSeq_ = 0;
volatile uint32_t MotorControlCoordinator::missedRoundCount_ = 0;
volatile uint32_t MotorControlCoordinator::lateComputeCount_ = 0;
volatile uint32_t MotorControlCoordinator::reusedOutputCount_ = 0;
volatile uint32_t MotorControlCoordinator::crossRoundComputeCount_ = 0;
volatile bool MotorControlCoordinator::roundReady_ = false;
volatile bool MotorControlCoordinator::outputsReady_ = false;
volatile bool MotorControlCoordinator::computeBusy_ = false;

void MotorControlCoordinator::init() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        roundCount_ = 0;
        requestedRound_ = 0;
        computedRound_ = 0;
        appliedRound_ = 0;
        currentSlot_ = 0;
        isrSlot_ = 0;
        roundStartUs_ = 0;
        computeSeq_ = 0;
        preparedSeq_ = 0;
        appliedSeq_ = 0;
        missedRoundCount_ = 0;
        lateComputeCount_ = 0;
        reusedOutputCount_ = 0;
        crossRoundComputeCount_ = 0;
        roundReady_ = false;
        outputsReady_ = false;
        computeBusy_ = false;
    }
}

void MotorControlCoordinator::snapshot(uint32_t &roundCount,
                                       uint32_t &requestedRound,
                                       uint32_t &computedRound,
                                       uint32_t &appliedRound,
                                       uint8_t &slot,
                                       uint8_t &computeSeq,
                                       uint8_t &appliedSeq,
                                       uint32_t &missedRoundCount,
                                       uint32_t &lateComputeCount,
                                       uint32_t &reusedOutputCount,
                                       uint32_t &crossRoundComputeCount,
                                       bool &computeBusy) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        roundCount = roundCount_;
        requestedRound = requestedRound_;
        computedRound = computedRound_;
        appliedRound = appliedRound_;
        slot = currentSlot_;
        computeSeq = computeSeq_;
        appliedSeq = appliedSeq_;
        missedRoundCount = missedRoundCount_;
        lateComputeCount = lateComputeCount_;
        reusedOutputCount = reusedOutputCount_;
        crossRoundComputeCount = crossRoundComputeCount_;
        computeBusy = computeBusy_;
    }
}

uint8_t MotorControlCoordinator::getComputeSeq() {
    uint8_t value;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        value = computeSeq_;
    }
    return value;
}

bool MotorControlCoordinator::isComputeBusy() {
    bool value;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        value = computeBusy_;
    }
    return value;
}

void MotorControlCoordinator::beginPidRoundIsr(DCMotor *motors, uint8_t motorCount, bool running) {
    if (running) {
        roundCount_++;
        uint32_t currentRound = roundCount_;
        if (outputsReady_ && computedRound_ == currentRound) {
            for (uint8_t i = 0; i < motorCount; i++) {
                motors[i].publishStagedOutputISR();
            }
            appliedSeq_ = preparedSeq_;
            appliedRound_ = currentRound;
            outputsReady_ = false;
        } else {
            reusedOutputCount_++;
        }
        if (roundReady_) {
            missedRoundCount_++;
        }
        requestedRound_ = currentRound + 1U;
        roundReady_ = true;
    } else {
        outputsReady_ = false;
        roundReady_ = false;
        requestedRound_ = roundCount_;
        computedRound_ = roundCount_;
        appliedRound_ = roundCount_;
        appliedSeq_ = preparedSeq_;
    }
}

uint16_t MotorControlCoordinator::servicePidIsrSlice(DCMotor *motors,
                                                     uint8_t motorCount,
                                                     bool running) {
    if (motors == nullptr || motorCount == 0U) {
        return 0U;
    }

    const uint8_t slot = isrSlot_;
    currentSlot_ = slot;

    if (slot == 0U) {
        roundStartUs_ = micros();
        beginPidRoundIsr(motors, motorCount, running);
    }

    motors[slot].latchFeedbackISR();
    motors[slot].update();

    uint16_t completedRoundUs = 0U;
    if (slot == (uint8_t)(motorCount - 1U) && running) {
        uint32_t elapsedUs = micros() - roundStartUs_;
        completedRoundUs = (elapsedUs > 0xFFFFUL) ? 0xFFFFU : (uint16_t)elapsedUs;
    }

    isrSlot_ = (uint8_t)((slot + 1U) % motorCount);
    currentSlot_ = isrSlot_;
    return completedRoundUs;
}

void MotorControlCoordinator::resetForNonRunningTask() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        computeBusy_ = false;
        roundReady_ = false;
        outputsReady_ = false;
        isrSlot_ = 0;
        currentSlot_ = 0;
        requestedRound_ = roundCount_;
        computedRound_ = roundCount_;
    }
}

bool MotorControlCoordinator::beginCompute(uint32_t &requestedRound, uint8_t &slotSnapshot) {
    bool shouldRun = false;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (roundReady_ && !computeBusy_) {
            requestedRound = requestedRound_;
            slotSnapshot = currentSlot_;
            roundReady_ = false;
            computeBusy_ = true;
            shouldRun = true;
        }
    }

    if (shouldRun && slotSnapshot == 0U) {
        lateComputeCount_++;
    }

    return shouldRun;
}

void MotorControlCoordinator::finishCompute(uint32_t requestedRound) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (roundCount_ >= requestedRound) {
            crossRoundComputeCount_++;
        }
        computeSeq_++;
        preparedSeq_ = computeSeq_;
        computedRound_ = requestedRound;
        outputsReady_ = true;
        computeBusy_ = false;
    }
}

#ifndef MOTOR_CONTROL_COORDINATOR_H
#define MOTOR_CONTROL_COORDINATOR_H

#include <Arduino.h>
#include <stdint.h>

class DCMotor;

class MotorControlCoordinator {
public:
    static void init();

    static void snapshot(uint32_t &roundCount,
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
                         bool &computeBusy);

    static uint8_t getComputeSeq();
    static bool isComputeBusy();

    /**
     * @brief Run one Timer1-driven DC control slice.
     *
     * This owns the round-robin slot index used by the 800 Hz Timer1 ISR:
     * - slot 0 starts a new 4-motor round and publishes prepared outputs
     * - the current motor's feedback is latched and cached output applied
     * - the next slot index is staged for the following ISR tick
     *
     * @param motors      DC motor array owned by the main firmware
     * @param motorCount  Number of valid entries in @p motors
     * @param running     True while the system is in RUNNING state
     * @return Completed round span in microseconds when the final slot closes
     *         a running round, or 0 otherwise.
     */
    static uint16_t servicePidIsrSlice(DCMotor *motors, uint8_t motorCount, bool running);

    static void resetForNonRunningTask();
    static bool beginCompute(uint32_t &requestedRound, uint8_t &slotSnapshot);
    static void finishCompute(uint32_t requestedRound);

private:
    static volatile uint32_t roundCount_;
    static volatile uint32_t requestedRound_;
    static volatile uint32_t computedRound_;
    static volatile uint32_t appliedRound_;
    static volatile uint8_t currentSlot_;
    static volatile uint8_t isrSlot_;
    static volatile uint32_t roundStartUs_;
    static volatile uint8_t computeSeq_;
    static volatile uint8_t preparedSeq_;
    static volatile uint8_t appliedSeq_;
    static volatile uint32_t missedRoundCount_;
    static volatile uint32_t lateComputeCount_;
    static volatile uint32_t reusedOutputCount_;
    static volatile uint32_t crossRoundComputeCount_;
    static volatile bool roundReady_;
    static volatile bool outputsReady_;
    static volatile bool computeBusy_;

    static void beginPidRoundIsr(DCMotor *motors, uint8_t motorCount, bool running);
};

#endif // MOTOR_CONTROL_COORDINATOR_H

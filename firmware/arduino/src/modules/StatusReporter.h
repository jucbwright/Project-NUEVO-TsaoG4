#ifndef STATUS_REPORTER_H
#define STATUS_REPORTER_H

#include <Arduino.h>

typedef void (*StatusReporterUartFaultSnapshotFn)(uint32_t &dor2, uint32_t &fe2);
typedef void (*StatusReporterMotorControlSnapshotFn)(uint32_t &roundCount,
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

class StatusReporter {
public:
    static void init(StatusReporterUartFaultSnapshotFn uartFaultSnapshot,
                     StatusReporterMotorControlSnapshotFn motorControlSnapshot);
    static void recordLoopGap();
    static void updateWindowPeaks();
    static void task();
    static void emitChunk();
    static void recordFlushTimingUs(uint16_t elapsedUs);
};

#endif // STATUS_REPORTER_H

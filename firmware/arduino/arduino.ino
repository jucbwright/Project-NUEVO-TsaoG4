/**
 * @file arduino.ino
 * @brief Main firmware for Arduino Mega 2560 educational robotics platform
 * @version 0.9.0
 *
 * Educational robotics platform firmware for MAE 162 course.
 * Provides real-time motor control, sensor integration, and
 * communication with Raspberry Pi 5 via TLV protocol over UART.
 *
 * Two-tier scheduling architecture:
 *
 *   Hard real-time (ISR-driven — unaffected by loop() blocking):
 *     TIMER3_OVF_vect  10 kHz  Stepper pulse generation (StepperManager)
 *
 *   Soft real-time (millis-based, runs in loop() with interrupts enabled):
 *     taskUART          50 Hz  UART RX/TX — loop() keeps USART2_RX_vect alive;
 *                              RX/TX stay out of the ISR path entirely
 *     taskMotors       200 Hz  DC control compute + apply + feedback refresh
 *     taskSafety      100 Hz  heartbeat/battery safety checks
 *     taskSensors     100 Hz  IMU, ultrasonic, voltage + input sampling
 *     taskUserIO        20 Hz  LED animations, NeoPixel status
 *
 * Initialization Order (setup):
 *  1. Debug serial (Serial0)
 *  2. Scheduler (millis-based soft scheduler)
 *  3. MessageCenter (Serial2 + TLV codec)
 *  4. SensorManager (I2C, ADC)
 *  5. UserIO (GPIO, NeoPixel)
 *  6. ServoController (PCA9685 via I2C)
 *  7. StepperManager (Timer3 — also starts stepper ISR)
 *  8. DC Motors (PWM pins, encoder counters)
 *  9. Attach encoder ISRs (via ISRScheduler helpers)
 * 10. Register periodic and fast-lane scheduler tasks
 * 11. Configure Timer1/Timer4 runtime timer hardware — LAST
 *
 * Main Loop:
 * - Scheduler::serviceFastLane() handles work-available tasks
 * - Scheduler::tickPeriodic() executes one highest-priority ready periodic task
 */

#include <util/atomic.h>

// ============================================================================
// INCLUDES
// ============================================================================

// Core configuration
#include "src/config.h"
#include "src/pins.h"
#include "src/Scheduler.h"
#include "src/ISRScheduler.h"
#include "src/SystemManager.h"
#include "src/utility.h"

// Communication
#include "src/modules/MessageCenter.h"
#include "src/modules/DebugLog.h"
#include "src/modules/DCMotorBringup.h"
#include "src/modules/LoopMonitor.h"
#include "src/modules/MotorControlCoordinator.h"
#include "src/modules/SafetyManager.h"
#include "src/modules/StatusReporter.h"
#include "src/messages/TLV_Payloads.h"

// DC motor control
#include "src/modules/EncoderCounter.h"
#include "src/modules/VelocityEstimator.h"
#include "src/drivers/DCMotor.h"

// Stepper and servo control
#include "src/modules/StepperManager.h"
#include "src/drivers/StepperMotor.h"
#include "src/drivers/ServoController.h"

// Sensors and user I/O
#include "src/modules/SensorManager.h"
#include "src/modules/UserIO.h"
#include "src/drivers/IMUDriver.h"
#include "src/drivers/NeoPixelDriver.h"

// ============================================================================
// Compiler Guards
// ============================================================================
#ifndef SERIAL_RX_BUFFER_SIZE
#error "SERIAL_RX_BUFFER_SIZE not defined. Add -DSERIAL_RX_BUFFER_SIZE=256 to compiler flags."
#elif SERIAL_RX_BUFFER_SIZE < 256
#error "SERIAL_RX_BUFFER_SIZE < 256, which is recommanded for 250 kbps."
#endif

#ifndef SERIAL_TX_BUFFER_SIZE
#error "SERIAL_TX_BUFFER_SIZE not defined. Add -DSERIAL_TX_BUFFER_SIZE=256 to compiler flags."
#elif SERIAL_TX_BUFFER_SIZE < 256
#error "SERIAL_TX_BUFFER_SIZE < 256, which is recommanded for 250 kbps."
#endif

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

// Encoder instances (2x or 4x mode per config.h)
#if ENCODER_1_MODE == ENCODER_2X
EncoderCounter2x encoder1;
#else
EncoderCounter4x encoder1;
#endif

#if ENCODER_2_MODE == ENCODER_2X
EncoderCounter2x encoder2;
#else
EncoderCounter4x encoder2;
#endif

#if ENCODER_3_MODE == ENCODER_2X
EncoderCounter2x encoder3;
#else
EncoderCounter4x encoder3;
#endif

#if ENCODER_4_MODE == ENCODER_2X
EncoderCounter2x encoder4;
#else
EncoderCounter4x encoder4;
#endif

// Velocity estimators (edge-time algorithm)
EdgeTimeVelocityEstimator velocityEst1;
EdgeTimeVelocityEstimator velocityEst2;
EdgeTimeVelocityEstimator velocityEst3;
EdgeTimeVelocityEstimator velocityEst4;

// Motor controller arrays
DCMotor dcMotors[NUM_DC_MOTORS];
static volatile uint32_t g_uartDor2Count = 0;
static volatile uint32_t g_uartFe2Count = 0;

static void snapshotUart2FaultCounts(uint32_t &dor2, uint32_t &fe2) {
  noInterrupts();
  dor2 = g_uartDor2Count;
  fe2 = g_uartFe2Count;
  interrupts();
}

static bool fastDrainUart() {
  int before = RPI_SERIAL.available();
  if (before <= 0) {
    return false;
  }
  MessageCenter::drainUart();
  return RPI_SERIAL.available() != before;
}

static bool fastDrainTx() {
  uint16_t before = MessageCenter::getTxPendingBytes();
  if (before == 0U) {
    return false;
  }
  MessageCenter::drainTx();
  return MessageCenter::getTxPendingBytes() != before;
}

static bool fastMotorCompute() {
  uint8_t seqBefore = MotorControlCoordinator::getComputeSeq();
  bool busyBefore = MotorControlCoordinator::isComputeBusy();
  taskMotors();
  return (MotorControlCoordinator::getComputeSeq() != seqBefore) ||
         (MotorControlCoordinator::isComputeBusy() != busyBefore);
}

static bool fastStatusChunk() {
#if STATUS_REPORTER_ENABLED
  uint16_t before = DebugLog::getQueuedBytes();
  StatusReporter::emitChunk();
  return DebugLog::getQueuedBytes() != before;
#else
  return false;
#endif
}

static bool fastDebugFlush() {
  uint16_t before = DebugLog::getQueuedBytes();
  if (before == 0U) {
    return false;
  }

  uint32_t flushStartUs = micros();
  DebugLog::flush();
  StatusReporter::recordFlushTimingUs(Utility::clampElapsedUs(micros() - flushStartUs));
  return DebugLog::getQueuedBytes() != before;
}

/**
 * @brief Timer1 overflow ISR — short round-robin DC apply slot.
 *
 * One motor is serviced per 800 Hz tick, giving each DC motor a 200 Hz apply
 * cadence while keeping the per-slice ISR body comfortably inside the UART
 * safety budget. This uses a fixed one-round pipeline:
 * - slot 0 of round N publishes outputs computed for round N
 * - loop() computes outputs for round N+1 during round N
 */
ISR(TIMER1_OVF_vect) {
  uint16_t t0 = Utility::readTimer1CounterTicks(); // Sample time stamp for the LoopMonitor
  bool running = (SystemManager::getState() == SYS_STATE_RUNNING);
  uint16_t pidRoundSpanUs =
      MotorControlCoordinator::servicePidIsrSlice(dcMotors, NUM_DC_MOTORS, running);

  if (running && pidRoundSpanUs > 0U) {
    LoopMonitor::recordPidRoundSpan(pidRoundSpanUs, true);
  } else if (!running) {
    LoopMonitor::recordPidRoundSpan(0, false);
  }

  LoopMonitor::record(SLOT_PID_ISR,
                      Utility::timerTicksToUs(
                          (uint16_t)(Utility::readTimer1CounterTicks() - t0)));
}

/**
 * @brief Timer3 overflow ISR — stepper pulse generation only.
 *
 * The actual per-stepper work lives in StepperManager::timerISR(). Keeping the
 * vector in main firmware makes the active hard-RT paths explicit.
 */
ISR(TIMER3_OVF_vect) {
  // Same Timer3 hardware-counter timing pattern as the Timer1 PID ISR above.
  uint16_t t0 = Utility::readTimer3CounterTicks();
  Utility::Uart2FaultEdges uartFaultEdges = Utility::sampleUart2FaultEdges();
  if (uartFaultEdges.dorRising) {
    g_uartDor2Count++;
  }
  if (uartFaultEdges.feRising) {
    g_uartFe2Count++;
  }

  StepperManager::timerISR();
  LoopMonitor::record(SLOT_STEPPER_ISR,
                      Utility::timerTicksToUs(
                          (uint16_t)(Utility::readTimer3CounterTicks() - t0)));
}

// ============================================================================
// SOFT TASK — taskUART (50 Hz, millis-based)
// ============================================================================

/**
 * @brief UART RX/TX task — runs in loop() so USART2_RX_vect stays enabled.
 *
 * Do NOT move into TIMER1_OVF_vect (see ISR comment above for explanation).
 * Registered at prior0 (highest) so it preempts all other soft tasks.
 */ 
void taskUART() {
#if DEBUG_PINS_ENABLED
  digitalWrite(DEBUG_PIN_UART_TASK, HIGH);
#endif

  uint32_t taskStartUs = micros();

#if DEBUG_PINS_ENABLED
  digitalWrite(DEBUG_PIN_UART_RX, HIGH);
#endif
  MessageCenter::processIncoming();
#if DEBUG_PINS_ENABLED
  digitalWrite(DEBUG_PIN_UART_RX, LOW);
  digitalWrite(DEBUG_PIN_UART_TX, HIGH);
#endif
  MessageCenter::processDeferred();
  MessageCenter::sendTelemetry();
#if DEBUG_PINS_ENABLED
  digitalWrite(DEBUG_PIN_UART_TX, LOW);
#endif

  uint16_t elapsed = Utility::clampElapsedUs(micros() - taskStartUs);
  MessageCenter::recordLoopTime(elapsed);
  LoopMonitor::record(SLOT_UART_TASK, elapsed);

#if DEBUG_PINS_ENABLED
  // A9 (UART_LATE): pulse when wall-clock > 20 ms. This fires due to ISR
  // preemption inflation, NOT actual UART slowness — the PID loop is unaffected.
  if (elapsed > UART_TASK_BUDGET_US) {
    digitalWrite(DEBUG_PIN_UART_LATE, HIGH);
    digitalWrite(DEBUG_PIN_UART_LATE, LOW);
  }
  digitalWrite(DEBUG_PIN_UART_TASK, LOW);
#endif
}

// ============================================================================
// SOFT TASK — taskSafety (100 Hz, millis-based)
// ============================================================================

/**
 * @brief Safety checks outside ISR context.
 *
 * SafetyManager owns only fault policy. User button / limit GPIO sampling runs
 * in taskSensors() so task ownership matches subsystem responsibility.
 */
void taskSafety() {
  SafetyManager::check();
}

// ============================================================================
// SOFT TASK — taskMotors (round-driven, loop-owned)
// ============================================================================

/**
 * @brief Run the full software DC compute round when a new ISR round completes.
 *
 * The compute round is triggered from slot 0 of the current round and prepares
 * the outputs that will be published at slot 0 of the next round.
 */
void taskMotors() {
  if (SystemManager::getState() != SYS_STATE_RUNNING) {
    MotorControlCoordinator::resetForNonRunningTask();
    return;
  }

  uint32_t requestedRound = 0;
  uint8_t slotSnapshot = 0;
  if (!MotorControlCoordinator::beginCompute(requestedRound, slotSnapshot)) {
    return;
  }
  (void)slotSnapshot;

  uint32_t t0 = micros();
  for (uint8_t i = 0; i < NUM_DC_MOTORS; i++) {
    dcMotors[i].service();
  }

  uint16_t elapsedUs = Utility::clampElapsedUs(micros() - t0);
  MotorControlCoordinator::finishCompute(requestedRound);
  LoopMonitor::record(SLOT_MOTOR_TASK, elapsedUs);
}

// ============================================================================
// SOFT TASK — taskSensors (100 Hz, millis-based)
// ============================================================================

/**
 * @brief Sensor dispatch task — runs in loop() so I2C/ADC never execute in ISR context.
 *
 * This is the main bring-up change relative to the older architecture: Timer4
 * still provides motor PWM, but its overflow interrupt is disabled. The same
 * 100 Hz task also refreshes the cached user button / limit switch states.
 */
void taskSensors() {
  uint32_t t0 = micros();
  SensorManager::tick();
  UserIO::sampleInputs();
  LoopMonitor::record(SLOT_SENSOR_ISR, Utility::clampElapsedUs(micros() - t0));
}

// ============================================================================
// SOFT TASK — taskUserIO (20 Hz, millis-based)
// ============================================================================

/**
 * @brief User I/O updates (20 Hz, soft scheduler)
 *
 * Runs user-controlled LED animations and queued NeoPixel state rendering.
 * Button / limit sampling is handled in taskSensors() at 100 Hz.
 */
void taskUserIO() {
  uint32_t t0 = micros();
  UserIO::serviceTask();
  LoopMonitor::record(SLOT_USERIO, Utility::clampElapsedUs(micros() - t0));
}

// ============================================================================
// ENCODER ISR TRAMPOLINES
// ============================================================================

/**
 * @brief Encoder ISR wrappers
 *
 * These are minimal ISR wrappers that forward calls to encoder objects.
 * Encoder ISRs must be global functions (not class methods) to use with
 * attachInterrupt().
 */

void encoderISR_M1_A() {
#if DEBUG_PINS_ENABLED
  digitalWrite(DEBUG_PIN_ENCODER_ISR, HIGH);
#endif
  encoder1.onInterruptA();
#if DEBUG_PINS_ENABLED
  digitalWrite(DEBUG_PIN_ENCODER_ISR, LOW);
#endif
}

void encoderISR_M1_B() {
  encoder1.onInterruptB();
}

void encoderISR_M2_A() {
  encoder2.onInterruptA();
}

void encoderISR_M2_B() {
  encoder2.onInterruptB();
}

// M3/M4 encoder pins (A14, A15, 11, 12) are on PCINT buses, not hardware
// INT pins. attachInterrupt() does not work for them on Mega, so the wrappers
// below are dispatched from the PCINT2/PCINT0 vectors.
void encoderISR_M3() {
  encoder3.onInterruptA();
}

void encoderISR_M4() {
  encoder4.onInterruptA();
}

ISR(PCINT2_vect) {
  encoderISR_M3();
}

ISR(PCINT0_vect) {
  encoderISR_M4();
}

// ============================================================================
// ARDUINO SETUP
// ============================================================================

void setup() {
  int8_t taskId = -1;
  int8_t fastTaskId = -1;

  // 1. Enter INIT before any module starts.
  SystemManager::init();

  // 2. Bring up debug serial and runtime-owned services.
  DEBUG_SERIAL_PORT.begin(DEBUG_BAUD_RATE);
  while (!DEBUG_SERIAL_PORT && millis() < 2000) {
    ; // Wait for serial port to connect (up to 2 seconds)
  }
  DebugLog::init();
  DebugLog::setPassthrough(true);
  LoopMonitor::init();
  MotorControlCoordinator::init();
  StatusReporter::init(snapshotUart2FaultCounts, MotorControlCoordinator::snapshot);
  Utility::printStartupBanner();

  // 3. Initialize scheduler and optional debug pins.
  DEBUG_SERIAL.println(F("[Setup] Initializing soft scheduler..."));
  Scheduler::init();
  Utility::initDebugPins();

  // 4. Initialize main modules and actuators.
  DEBUG_SERIAL.println(F("[Setup] Initializing UART communication..."));
  MessageCenter::init();

  DEBUG_SERIAL.println(F("[Setup] Initializing sensors..."));
  SensorManager::init();

  DEBUG_SERIAL.println(F("[Setup] Initializing user I/O..."));
  UserIO::init();

#if SERVO_CONTROLLER_ENABLED
  DEBUG_SERIAL.println(F("[Setup] Initializing servo controller..."));
  ServoController::init();
  DEBUG_SERIAL.println(F("  - PCA9685 initialized (50Hz PWM)"));
#endif

  DEBUG_SERIAL.println(F("[Setup] Initializing stepper motors..."));
  StepperManager::init();

  // 5. Initialize DC motors, encoders, and their interrupt attachment.
  DEBUG_SERIAL.println(F("[Setup] Initializing DC motors and encoders..."));
  DEBUG_SERIAL.print(F("  - Encoder resolution: "));
  DEBUG_SERIAL.print(DCMotorBringup::countsPerRev());
  DEBUG_SERIAL.println(F(" counts/rev"));
  DCMotorBringup::initAll(dcMotors,
                          encoder1, velocityEst1,
                          encoder2, velocityEst2,
                          encoder3, velocityEst3,
                          encoder4, velocityEst4);
  DEBUG_SERIAL.println(F("  - 4 DC motors initialized"));

  DEBUG_SERIAL.println(F("[Setup] Attaching encoder interrupts..."));
  ISRScheduler::attachDcEncoderInterrupts(encoderISR_M1_A,
                                          encoderISR_M1_B,
                                          encoderISR_M2_A,
                                          encoderISR_M2_B);
  DEBUG_SERIAL.println(F("  - Motor 1/2 encoder ISRs attached (hardware INT)"));
  ISRScheduler::attachDcEncoderPcints();
  DEBUG_SERIAL.println(F("  - Motor 3/4 encoder ISRs attached (PCINT)"));

  // 6. Register cooperative fast-lane and periodic soft tasks.
  DEBUG_SERIAL.println(F("[Setup] Registering scheduler tasks..."));

  fastTaskId = Scheduler::registerFastTask(fastDrainUart, 0);
  if (fastTaskId >= 0) {
    DEBUG_SERIAL.print(F("  - Fast UART RX: Task #"));
    DEBUG_SERIAL.println(fastTaskId);
  }

  fastTaskId = Scheduler::registerFastTask(fastDrainTx, 1);
  if (fastTaskId >= 0) {
    DEBUG_SERIAL.print(F("  - Fast UART TX: Task #"));
    DEBUG_SERIAL.println(fastTaskId);
  }

  fastTaskId = Scheduler::registerFastTask(fastMotorCompute, 2);
  if (fastTaskId >= 0) {
    DEBUG_SERIAL.print(F("  - Fast MotorCompute: Task #"));
    DEBUG_SERIAL.println(fastTaskId);
  }

#if STATUS_REPORTER_ENABLED
  fastTaskId = Scheduler::registerFastTask(fastStatusChunk, 3);
  if (fastTaskId >= 0) {
    DEBUG_SERIAL.print(F("  - Fast StatusChunk: Task #"));
    DEBUG_SERIAL.println(fastTaskId);
  }
#endif

  fastTaskId = Scheduler::registerFastTask(fastDebugFlush, 4);
  if (fastTaskId >= 0) {
    DEBUG_SERIAL.print(F("  - Fast DebugFlush: Task #"));
    DEBUG_SERIAL.println(fastTaskId);
  }

  taskId = Scheduler::registerTask(taskUART, 1000 / UART_COMMS_FREQ_HZ, 0);
  if (taskId >= 0) {
    DEBUG_SERIAL.print(F("  - UART: Task #"));
    DEBUG_SERIAL.print(taskId);
    DEBUG_SERIAL.print(F(" @ "));
    DEBUG_SERIAL.print(1000 / UART_COMMS_FREQ_HZ);
    DEBUG_SERIAL.println(F("ms (50Hz)"));
  }

  taskId = Scheduler::registerTask(taskSafety, 1000 / SENSOR_UPDATE_FREQ_HZ, 1);
  if (taskId >= 0) {
    DEBUG_SERIAL.print(F("  - Safety: Task #"));
    DEBUG_SERIAL.print(taskId);
    DEBUG_SERIAL.print(F(" @ "));
    DEBUG_SERIAL.print(1000 / SENSOR_UPDATE_FREQ_HZ);
    DEBUG_SERIAL.println(F("ms (100Hz)"));
  }

  DEBUG_SERIAL.println(F("  - Motors: round-driven in loop() from Timer1 slot-3 flag"));

  taskId = Scheduler::registerTask(taskSensors, 1000 / SENSOR_UPDATE_FREQ_HZ, 3);
  if (taskId >= 0) {
    DEBUG_SERIAL.print(F("  - Sensors: Task #"));
    DEBUG_SERIAL.print(taskId);
    DEBUG_SERIAL.print(F(" @ "));
    DEBUG_SERIAL.print(1000 / SENSOR_UPDATE_FREQ_HZ);
    DEBUG_SERIAL.println(F("ms (100Hz)"));
  }

  taskId = Scheduler::registerTask(taskUserIO, 1000 / USER_IO_FREQ_HZ, 4);
  if (taskId >= 0) {
    DEBUG_SERIAL.print(F("  - User I/O: Task #"));
    DEBUG_SERIAL.print(taskId);
    DEBUG_SERIAL.print(F(" @ "));
    DEBUG_SERIAL.print(1000 / USER_IO_FREQ_HZ);
    DEBUG_SERIAL.println(F("ms"));
  }

#if STATUS_REPORTER_ENABLED
  taskId = Scheduler::registerTask(StatusReporter::task, 1000 / STATUS_REPORT_HZ, 5);
  if (taskId >= 0) {
    DEBUG_SERIAL.print(F("  - Status Reporter: Task #"));
    DEBUG_SERIAL.print(taskId);
    DEBUG_SERIAL.print(F(" @ "));
    DEBUG_SERIAL.print(1000 / STATUS_REPORT_HZ);
    DEBUG_SERIAL.println(F("ms"));
  }
#else
  DEBUG_SERIAL.println(F("  - Status Reporter: disabled"));
#endif

  // 7. Move into IDLE before enabling hard real-time services.
  if (SystemManager::triggerBootCompleted()) {
    DEBUG_SERIAL.println(F("[Setup] System state -> IDLE"));
  } else {
    DEBUG_SERIAL.println(F("[Setup] System state transition to IDLE rejected"));
  }

  // 8. Start hard real-time timer services last.
  DEBUG_SERIAL.println(F("[Setup] Starting hard real-time ISRs (Timer1 + Timer3; Timer4 PWM only)..."));
  noInterrupts();
  ISRScheduler::configureTimer1DcSlotISR();
  ISRScheduler::configureTimer4PwmOnly();
  interrupts();
  UserIO::syncOutputs();
  Utility::printStartupSummary();
  DebugLog::setPassthrough(false);
}

// ============================================================================
// ARDUINO MAIN LOOP
// ============================================================================

void loop() {
  StatusReporter::recordLoopGap();
  StatusReporter::updateWindowPeaks();
  Scheduler::serviceFastLane();
  Scheduler::tickPeriodic();
  Scheduler::serviceFastLane();
}

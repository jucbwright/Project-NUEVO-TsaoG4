#ifndef UTILITY_H
#define UTILITY_H

#include <Arduino.h>
#include <stdint.h>

namespace Utility {

struct Uart2FaultEdges {
    bool dorRising;
    bool feRising;
};

/**
 * @brief Clamp an elapsed-time measurement to the 16-bit range used by reports.
 */
uint16_t clampElapsedUs(uint32_t elapsedUs);

/**
 * @brief Read the current Timer1 hardware counter value.
 *
 * Timer1 is clocked at F_CPU/8 in this firmware, so the raw counter advances
 * every 0.5 us on the Mega 2560.
 */
uint16_t readTimer1CounterTicks();

/**
 * @brief Read the current Timer3 hardware counter value.
 *
 * Timer3 is clocked at F_CPU/8 in this firmware, so the raw counter advances
 * every 0.5 us on the Mega 2560.
 */
uint16_t readTimer3CounterTicks();

/**
 * @brief Convert Timer1/Timer3 tick counts to microseconds.
 *
 * With F_CPU/8 timer clocks on the Mega 2560, two timer ticks are one
 * microsecond, so this helper converts the raw register delta into a report-
 * friendly microsecond value.
 */
uint16_t timerTicksToUs(uint16_t ticks);

/**
 * @brief Sample UART2 hardware error flags and report new rising edges.
 *
 * The Mega exposes UART receive fault bits in the USART2 status register
 * (`UCSR2A`):
 * - `DOR2`: data overrun
 * - `FE2` : framing error
 *
 * This helper hides the AVR register names and keeps track of the previous
 * sampled state so ISRs can count each newly observed fault once.
 */
Uart2FaultEdges sampleUart2FaultEdges();

/**
 * @brief Print the startup banner to the debug console.
 */
void printStartupBanner();

/**
 * @brief Configure optional oscilloscope debug pins.
 */
void initDebugPins();

/**
 * @brief Print the firmware startup scheduling summary to the debug console.
 */
void printStartupSummary();

} // namespace Utility

#endif // UTILITY_H

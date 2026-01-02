/* 
  optolinkGWG.h - Connect Viessmann heating devices via Optolink GWG protocol to ESPhome

  Copyright (c) 2025 Uwe Wendt

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "vitoconnect_optolink.h"

namespace esphome {
namespace vitoconnect {

/**
 * @brief Protocol implementation class for the Optolink (GWG).
 *
 * This class implements the GWG protocol variant of the Viessmann Optolink.
 *
 * Protocol concept (simplified, as observed in practice):
 * - The controller signals readiness by sending 0x05 (READY).
 * - The client must acknowledge READY by sending 0x01 (ACK).
 * - A request frame (READ/WRITE) is then transmitted.
 *
 * Performance improvement (burst mode):
 * - After a successful response, the next queued request is sent immediately
 *   without waiting for another READY (0x05).
 * - During burst mode, no further ACK (0x01) is sent, because 0x01 is only
 *   the ACK for a READY (0x05) event.
 *
 * GWG limitation:
 * - GWG only supports 1-byte addresses (<= 0xFF).
 * - Any queued datapoint with a larger address must be discarded in SEND state.
 */
class OptolinkGWG : public Optolink {
 public:
  explicit OptolinkGWG(uart::UARTDevice* uart);

  void begin();
  void loop();

 private:
  enum OptolinkState : uint8_t {
    INIT,
    IDLE,
    SEND,
    RECEIVE,
    UNDEF
  } _state;

  void _init();
  void _idle();
  void _send();
  void _receive();

  // Drain UART RX buffer to remove delayed or stale bytes.
  void _drain_uart_();

  // Drop invalid datapoints (e.g. address out of range for GWG) from the queue.
  // Returns true if a valid datapoint is available afterwards, false otherwise.
  bool _drop_invalid_queue_entries_();

  // Timestamp of last meaningful activity (connection watchdog)
  uint32_t _lastMillis;

  // Timestamp when the current request was sent (total RX timeout)
  uint32_t _sendMillis;

  // Timestamp of the last received byte (inter-byte timeout)
  uint32_t _lastRxMillis;

  // Timestamp when READY (0x05) was received (diagnostics)
  uint32_t _readyMillis;

  // Indicates whether we are inside a burst sequence (send next requests immediately)
  bool _burstActive;

  bool _write;

  // Receive buffer for protocol responses
  uint8_t _rcvBuffer[MAX_DP_LENGTH];
  size_t _rcvBufferLen;
  size_t _rcvLen;
};

}  // namespace vitoconnect
}  // namespace esphome

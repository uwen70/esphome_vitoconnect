/* 
  optolinkGWG.cpp - Connect Viessmann heating devices via Optolink GWG protocol to ESPhome

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

/*
 * =========================
 * GWG Protocol State Diagram
 * =========================
 *
 *                          (power-on / reset)
 *                                 |
 *                                 v
 *                              +------+
 *                              | INIT |
 *                              +------+
 *                                 |
 *                     wait for READY (0x05)
 *                                 |
 *                                 v
 *                              +------+
 *                              | IDLE |
 *                              +------+
 *                                 |
 *              READY (0x05) & queue not empty
 *                -> send ACK (0x01)
 *                                 |
 *                                 v
 *                              +------+
 *                              | SEND |
 *                              +------+
 *                                 |
 *                   send READ / WRITE request
 *                   (CB / C8 frame, no ACK here)
 *                                 |
 *                                 v
 *                            +----------+
 *                            | RECEIVE  |
 *                            +----------+
 *                                 |
 *          +----------------------+----------------------+
 *          |                                             |
 *   full response received                        timeout / error
 *          |                                             |
 *          v                                             v
 *   queue not empty & burst active                     +------+
 *          |                                           | INIT |
 *          |                                           +------+
 *          v
 *      +------+
 *      | SEND |   (burst mode: next request immediately,
 *      +------+    no READY, no ACK)
 *
 *          |
 *          v
 *   queue empty or burst ended
 *          |
 *          v
 *      +------+
 *      | IDLE |   (wait for next READY 0x05)
 *      +------+
 *
 *
 * Notes:
 * - ACK (0x01) is sent ONLY in IDLE as reaction to READY (0x05).
 * - SEND never sends ACK, only request frames.
 * - Burst mode accelerates polling by chaining SEND->RECEIVE
 *   without waiting for additional READY signals.
 * - Any timeout or protocol error resets the state machine to INIT.
 */

#include "vitoconnect_optolinkGWG.h"

namespace esphome {
namespace vitoconnect {

static const char *TAG = "vitoconnect";

// Recommended timings for GWG / 4800 baud:
// - The complete response must arrive within this time window.
// - With burst mode enabled, responses are usually fast once communication is active.
//   Keep this value conservative to avoid false timeouts caused by scheduler jitter.
static constexpr uint32_t GWG_RX_TOTAL_TIMEOUT_MS = 800UL;

// Maximum allowed gap between two consecutive bytes of a response.
// Larger gaps indicate a broken or aborted frame.
static constexpr uint32_t GWG_RX_INTERBYTE_TIMEOUT_MS = 60UL;

OptolinkGWG::OptolinkGWG(uart::UARTDevice* uart) :
  Optolink(uart),
  _state(UNDEF),
  _lastMillis(0),
  _sendMillis(0),
  _lastRxMillis(0),
  _readyMillis(0),
  _burstActive(false),
  _write(false),
  _rcvBuffer{0},
  _rcvBufferLen(0),
  _rcvLen(0) {}

void OptolinkGWG::begin() {
  _state = INIT;
  _lastMillis = millis();
  _sendMillis = 0;
  _lastRxMillis = 0;
  _readyMillis = 0;
  _burstActive = false;
}

// Helper function:
// Drain the UART RX buffer completely.
// This is required after protocol errors or timeouts to ensure that
// delayed bytes (e.g. late 0x05 ready signals) are not misinterpreted
// as part of a new response.
void OptolinkGWG::_drain_uart_() {
  while (_uart->available()) {
    (void) _uart->read();
  }
}

// Helper function:
// GWG supports only 1-byte addresses (<= 0xFF). Other protocols may use 2-byte addresses.
// If such entries are present in the queue, they must be discarded in GWG mode.
// This function removes invalid entries until either the queue is empty or the front is valid.
bool OptolinkGWG::_drop_invalid_queue_entries_() {
  while (_queue.size() > 0) {
    OptolinkDP *dp = _queue.front();

    // For GWG, only the low byte is usable, therefore any address > 0xFF is invalid here.
    if (dp->address <= 0xFF) {
      return true;  // front entry is valid
    }

    // Discard invalid entry and continue with the next.
    ESP_LOGW(TAG, "GWG: discarding datapoint with unsupported address 0x%04X", (unsigned) dp->address);

    // Remove the item from the queue.
    _queue.pop();

    // IMPORTANT:
    // Only delete dp here if this class owns the datapoint allocation.
    // If the base class or another component manages lifetime, do NOT delete.
    // delete dp;
  }

  return false;  // queue empty
}

void OptolinkGWG::loop() {
  switch (_state) {
    case INIT:
      _init();
      break;
    case IDLE:
      _idle();
      break;
    case SEND:
      _send();
      break;
    case RECEIVE:
      _receive();
      break;
    default:
      break;
  }

  // Connection watchdog:
  // If there are pending datapoints in the queue but no successful
  // communication for a prolonged time, reset the protocol state.
  // This protects against deadlocks caused by lost sync conditions.
  if (_queue.size() > 0 && millis() - _lastMillis > 5000UL) {
    _tryOnError(TIMEOUT);
    _state = INIT;
    _burstActive = false;
    _drain_uart_();
    _lastMillis = millis();
  }
}

void OptolinkGWG::_init() {
  // INIT state:
  // The goal is to synchronize with the controller.
  // We wait for the READY byte (0x05) and discard everything else.
  if (_uart->available()) {
    uint8_t b = _uart->read();
    if (b == 0x05) {
      _state = IDLE;
      _lastMillis = millis();
      _readyMillis = _lastMillis;
      return;
    }
  }
  // Stay in INIT until a valid READY byte is received.
}

void OptolinkGWG::_idle() {
  // IDLE state:
  // The controller signals readiness by sending 0x05 (READY).
  //
  // IMPORTANT:
  // - 0x01 is the ACK for READY (0x05) and must be sent immediately after receiving 0x05,
  //   otherwise the controller may ignore the upcoming request.
  // - The first request of a polling sequence is started from this state.
  // - After that, burst mode may take over (SEND is triggered directly after RECEIVE).
  if (_uart->available()) {
    uint8_t b = _uart->read();

    if (b == 0x05) {
      _readyMillis = millis();
      _lastMillis = _readyMillis;

      if (_queue.size() > 0) {
        // Start (or restart) a burst sequence.
        _burstActive = true;

        // ACK the READY byte.
        // This ACK must be sent only as reaction to 0x05, not for burst requests.
        const uint8_t ack[1] = {0x01};
        _uart->write_array(ack, sizeof(ack));

        // Proceed to SEND state to transmit the actual request frame.
        _state = SEND;
      } else {
        // No pending requests: stay in IDLE, but do not enter burst mode.
        _burstActive = false;
      }
      return;
    }

    // Any other byte received in IDLE is unexpected and ignored.
    ESP_LOGD(TAG, "Received unexpected byte 0x%02X in IDLE", b);
    return;
  }

  // Remain in IDLE while waiting for READY.
}

void OptolinkGWG::_send() {
  // SEND state:
  // This state transmits the request frame (READ/WRITE).
  //
  // GWG limitation:
  // - GWG supports only 1-byte addresses (<= 0xFF).
  // - If a queued datapoint uses a larger address (e.g. 2-byte addresses for KW),
  //   it must be discarded and the next queue entry must be tried immediately.
  //
  // IMPORTANT:
  // - No ACK (0x01) is sent here. ACK belongs exclusively to IDLE as a reaction to READY (0x05).
  // - In burst mode, SEND is entered without a new READY (0x05) event and therefore without ACK.
  // - Before sending, we drain any stale RX bytes to avoid mixing delayed bytes into the response.

  // Drop invalid datapoints (e.g. address out of range for GWG).
  // If no valid datapoint remains, end burst and return to IDLE.
  if (!_drop_invalid_queue_entries_()) {
    _burstActive = false;
    _state = IDLE;
    return;
  }

  _drain_uart_();

  uint8_t buff[MAX_DP_LENGTH + 4];
  OptolinkDP* dp = _queue.front();

  const uint8_t length = dp->length;
  const uint16_t address = dp->address;  // guaranteed <= 0xFF by _drop_invalid_queue_entries_()

  if (dp->write) {
    // WRITE command format:
    // C8 <addr> <len> 04 <value...>
    buff[0] = 0xC8;
    buff[1] = address & 0xFF;
    buff[2] = length;
    buff[3] = 0x04;
    memcpy(&buff[4], dp->data, length);

    // Only an ACK (0x00) is expected as response.
    _rcvLen = 1;
    _uart->write_array(buff, 4 + length);
  } else {
    // READ command format:
    // CB <addr> <len> 04
    buff[0] = 0xCB;
    buff[1] = address & 0xFF;
    buff[2] = length;
    buff[3] = 0x04;

    // Expected response length equals the requested data length.
    _rcvLen = length;
    _uart->write_array(buff, 4);
  }

  _rcvBufferLen = 0;
  memset(_rcvBuffer, 0, sizeof(_rcvBuffer));

  // Store timestamps for timeout handling and diagnostics.
  _sendMillis = millis();
  _lastRxMillis = _sendMillis;
  _lastMillis = _sendMillis;

  // Diagnostic information:
  // Measures how long it took from READY (0x05) to SEND (only meaningful for the first request in a burst).
  // In burst mode, READY->SEND delay will typically be large or irrelevant.
  ESP_LOGD(TAG, "READY->SEND delay: %lu ms",
           (unsigned long)(_sendMillis - _readyMillis));

  _state = RECEIVE;
}

void OptolinkGWG::_receive() {
  // RECEIVE state:
  // Collect response bytes until the expected response length is met or a timeout occurs.
  while (_uart->available() != 0) {
    uint8_t b = _uart->read();

    // Protect against buffer overflow.
    if (_rcvBufferLen >= sizeof(_rcvBuffer)) {
      ESP_LOGW(TAG, "RX buffer overflow (len=%d), resetting", (int)_rcvBufferLen);
      _rcvBufferLen = 0;
      memset(_rcvBuffer, 0, sizeof(_rcvBuffer));
      _state = INIT;
      _burstActive = false;
      _drain_uart_();
      _lastMillis = millis();
      return;
    }

    _rcvBuffer[_rcvBufferLen++] = b;
    _lastRxMillis = millis();
  }

  // Case 1: Complete response received.
  if (_rcvBufferLen == _rcvLen) {
    uint32_t rx_time = millis() - _sendMillis;
    OptolinkDP* dp = _queue.front();

    ESP_LOGD(TAG, "RX complete: addr=0x%02X len=%d time=%lu ms",
             (unsigned)dp->address,
             (int)_rcvBufferLen,
             (unsigned long)rx_time);

    // Forward data to datapoint handler.
    // Note: Typically _tryOnData() will pop the datapoint from the queue (depending on base implementation).
    _tryOnData(_rcvBuffer, _rcvBufferLen);

    _lastMillis = millis();

    // Burst mode behavior:
    // If further datapoints are queued, send the next request immediately.
    // This avoids waiting for another READY (0x05) and significantly speeds up polling.
    //
    // IMPORTANT:
    // - No ACK (0x01) is sent for burst requests, because 0x01 is only the ACK for READY (0x05).
    if (_burstActive && _queue.size() > 0) {
      _state = SEND;
      return;
    }

    // End of burst: go back to IDLE and wait for the next READY (0x05).
    _burstActive = false;
    _state = IDLE;
    return;
  }

  // Case 2: Inter-byte timeout.
  // Some bytes arrived, but the gap between them was too large.
  if (_rcvBufferLen > 0 &&
      millis() - _lastRxMillis > GWG_RX_INTERBYTE_TIMEOUT_MS) {
    ESP_LOGD(TAG, "Inter-byte timeout: got %d expected %d",
             (int)_rcvBufferLen, (int)_rcvLen);
    _rcvBufferLen = 0;
    memset(_rcvBuffer, 0, sizeof(_rcvBuffer));
    _state = INIT;
    _burstActive = false;
    _drain_uart_();
    _lastMillis = millis();
    return;
  }

  // Case 3: Total response timeout.
  // The response did not complete within the allowed time window.
  if (millis() - _sendMillis > GWG_RX_TOTAL_TIMEOUT_MS) {
    ESP_LOGD(TAG, "RX total timeout: got %d expected %d waited=%lu ms",
             (int)_rcvBufferLen,
             (int)_rcvLen,
             (unsigned long)(millis() - _sendMillis));
    _rcvBufferLen = 0;
    memset(_rcvBuffer, 0, sizeof(_rcvBuffer));
    _state = INIT;
    _burstActive = false;
    _drain_uart_();
    _lastMillis = millis();
    return;
  }

  // Otherwise: wait for more bytes.
}

}  // namespace vitoconnect
}  // namespace esphome

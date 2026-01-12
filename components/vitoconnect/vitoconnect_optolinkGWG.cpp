/* optolinkGWG.cpp - Connect Viessmann heating devices via Optolink GWG protocol to ESPhome
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
   along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

/*
 * =========================
 * GWG Protocol State Diagram
 * =========================
 *
 * (power-on / reset)
 *      |
 *      v
 *   +------+         wait for READY (0x05)
 *   | INIT |----------------------------------+
 *   +------+                                  |
 *      | READY (0x05)                         |
 *      v                                      |
 *   +------+                                  |
 *   | IDLE |  READY (0x05) & queue not empty  |
 *   +------+----------------------------------+
 *      | send ACK (0x01) as reaction to READY
 *      v
 *   +------+    send READ / WRITE request frame
 *   | SEND |----------------------------------+
 *   +------+                                  |
 *      |                                      |
 *      v                                      |
 *   +----------+   full response received     |
 *   | RECEIVE  |------------------------------+
 *   +----------+
 *      |
 *      | if queue not empty & burst active:
 *      |    -> SEND immediately (burst mode, no READY, no ACK)
 *      |
 *      v
 *    IDLE (wait for next READY 0x05)
 *
 * Notes:
 * - ACK (0x01) is sent ONLY in IDLE as reaction to READY (0x05).
 * - SEND never sends ACK, only request frames.
 * - Burst mode accelerates polling by chaining SEND->RECEIVE without waiting for READY.
 * - Any timeout or protocol error resets the state machine to INIT.
 */

#include "vitoconnect_optolinkGWG.h"

namespace esphome {
namespace vitoconnect {

static const char *TAG = "vitoconnect";

// Recommended timings for GWG / 4800 baud:
// - The complete response must arrive within this time window.
// - With burst mode enabled, responses are usually fast once communication is active.
// Keep this value conservative to avoid false timeouts caused by scheduler jitter.
static constexpr uint32_t GWG_RX_TOTAL_TIMEOUT_MS = 800UL;

// Maximum allowed gap between two consecutive bytes of a response.
// Larger gaps indicate a broken or aborted frame.
static constexpr uint32_t GWG_RX_INTERBYTE_TIMEOUT_MS = 80UL;

OptolinkGWG::OptolinkGWG(uart::UARTDevice *uart)
    : Optolink(uart),
      _state(UNDEF),
      _lastMillis(0),
      _sendMillis(0),
      _lastRxMillis(0),
      _readyMillis(0),
      _burstActive(false),
      _write(false),
      _rcvBuffer{0},
      _rcvBufferLen(0),
      _rcvLen(0),
      _eprom_read_active(false),
      _eprom_base_addr(0),
      _eprom_total_len(0),
      _eprom_index(0),
      _eprom_payload{0} {}

void OptolinkGWG::begin() {
  _state = INIT;
  _lastMillis = millis();
  _sendMillis = 0;
  _lastRxMillis = 0;
  _readyMillis = 0;
  _burstActive = false;

  // Reset EEPROM multi-read tracking.
  _eprom_read_active = false;
  _eprom_base_addr = 0;
  _eprom_total_len = 0;
  _eprom_index = 0;
  memset(_eprom_payload, 0, sizeof(_eprom_payload));
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

bool OptolinkGWG::_is_eeprom_read_(const OptolinkDP *dp) const {
  if (dp == nullptr) return false;
  const uint8_t func = (dp->address >> 8) & 0xFF;
  return (func == 0x05 && dp->write == false);
}

// Helper: validate function + direction and discard invalid queue entries.
//
// Address encoding:
// MSB: function
// LSB: physical address (0x00..0xFF)
//
// If function==0x00 -> legacy behavior (physical read/write by write flag).
// If function!=0x00 -> telegram byte is selected by function, and write flag must match direction.
// Otherwise the datapoint is discarded.
bool OptolinkGWG::_drop_invalid_queue_entries_() {
  while (_queue.size() > 0) {
    OptolinkDP *dp = _queue.front();

    // GWG supports only 1-byte physical addresses.
    // In legacy mode (MSB==0x00), dp->address must therefore be <= 0x00FF.
    // If somebody accidentally queues KW-style 2-byte addresses, we discard them here
    // to prevent silent, confusing behavior.
    const uint8_t func = (dp->address >> 8) & 0xFF;
    const uint8_t addr = dp->address & 0xFF;

    if (func == 0x00 && dp->address > 0x00FF) {
      ESP_LOGW(TAG,
               "GWG: discarding datapoint with 2-byte physical address 0x%04X (GWG supports only <=0x00FF)",
               (unsigned) dp->address);
      _queue.pop();
      // delete dp; // only if this class owns dp allocation
      continue;
    }

    // Determine whether function is supported and whether dp->write matches.
    bool supported = true;
    bool requires_write = false;
    bool requires_read = false;

    if (func == 0x00) {
      // Legacy mode: direction is controlled solely by dp->write.
      supported = true;
    } else {
      // Function table from user (MSB of address).
      // We only validate direction here; actual telegram byte is selected in _send().
      switch (func) {
        case 0x01: requires_read = true;  break;  // VIRTUAL READ
        case 0x02: requires_write = true; break;  // VIRTUAL WRITE
        case 0x03: requires_read = true;  break;  // PHYSICAL READ
        case 0x04: requires_write = true; break;  // PHYSICAL WRITE
        case 0x05: requires_read = true;  break;  // EEPROM READ
        case 0x06: requires_write = true; break;  // EEPROM WRITE
        case 0x49: requires_read = true;  break;  // PHYSICAL XRAM READ
        case 0x50: requires_write = true; break;  // PHYSICAL XRAM WRITE
        case 0x51: requires_read = true;  break;  // PHYSICAL PORT READ
        case 0x52: requires_write = true; break;  // PHYSICAL PORT WRITE
        case 0x53: requires_read = true;  break;  // PHYSICAL BE READ
        case 0x54: requires_write = true; break;  // PHYSICAL BE WRITE
        case 0x65: requires_read = true;  break;  // PHYSICAL KMBUS RAM READ (read-only)
        case 0x67: requires_read = true;  break;  // PHYSICAL KMBUS EEPROM READ (read-only)
        default:
          supported = false;
          break;
      }
    }

    if (!supported) {
      ESP_LOGW(TAG,
               "GWG: discarding datapoint with unsupported function MSB=0x%02X addr=0x%02X full=0x%04X",
               func, addr, (unsigned) dp->address);
      _queue.pop();
      // delete dp; // only if this class owns dp allocation
      continue;
    }

    // If function != 0x00, enforce direction match with dp->write.
    if (func != 0x00) {
      if ((requires_write && !dp->write) || (requires_read && dp->write)) {
        ESP_LOGW(TAG,
                 "GWG: discarding datapoint due to direction mismatch: MSB=0x%02X addr=0x%02X full=0x%04X write=%d",
                 func, addr, (unsigned) dp->address, (int) dp->write);
        _queue.pop();
        // delete dp; // only if this class owns dp allocation
        continue;
      }
    }

    // Valid entry found.
    return true;
  }
  return false;
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

    // Also reset any EEPROM multi-read in progress,
    // because the queued dp may now be out-of-sync with the bus.
    _eprom_read_active = false;
    _eprom_total_len = 0;
    _eprom_index = 0;

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
  // - Drop invalid queue entries (unsupported function or direction mismatch).
  // - Build request frame based on function (MSB) + write flag.
  // - Use only the LSB as physical address (GWG supports 1-byte addresses).
  // - No ACK (0x01) is sent here; ACK belongs exclusively to IDLE reacting to READY (0x05).
  //
  // Frame formats are kept consistent with the legacy implementation:
  // - READ :  <TYPE> <ADDR> <LEN> 04
  // - WRITE:  <TYPE> <ADDR> <LEN> 04 <DATA...>
  //
  // IMPORTANT:
  // - In burst mode, SEND is entered without a new READY (0x05) event and therefore without ACK.
  // - Before sending, we drain any stale RX bytes to avoid mixing delayed bytes into the response.

  // Drop invalid datapoints.
  // If no valid datapoint remains, end burst and return to IDLE.
  if (!_drop_invalid_queue_entries_()) {
    _burstActive = false;
    _state = IDLE;
    return;
  }

  _drain_uart_();

  OptolinkDP *dp = _queue.front();
  const uint8_t func = (dp->address >> 8) & 0xFF;
  const uint8_t addr = dp->address & 0xFF;

  // EEPROM read special case:
  // - EEPROM accesses turned out to work only byte-wise.
  // - Even if the YAML requests N bytes, we must split into N reads of len=1.
  //
  // The dp->length remains the "logical" requested length, but the actual bus request length
  // for EEPROM READ is always 1 byte.
  const bool is_eeprom_read = _is_eeprom_read_(dp);
  const uint8_t logical_len = dp->length;

  // Select telegram byte (TYPE) based on function (MSB) and write flag.
  uint8_t type = 0x00;
  bool type_ok = true;

  if (func == 0x00) {
    // Legacy behavior: physical read/write selected by dp->write.
    type = dp->write ? 0xC8 : 0xCB;
  } else {
    // Match telegram byte to function table.
    // Direction mismatch is already filtered in _drop_invalid_queue_entries_(),
    // but we keep this switch strict and self-contained.
    switch (func) {
      case 0x01: type = 0xC7; break;  // VIRTUAL READ
      case 0x02: type = 0xC4; break;  // VIRTUAL WRITE
      case 0x03: type = 0xCB; break;  // PHYSICAL READ
      case 0x04: type = 0xC8; break;  // PHYSICAL WRITE
      case 0x05: type = 0xAE; break;  // EEPROM READ
      case 0x06: type = 0xAD; break;  // EEPROM WRITE
      case 0x49: type = 0xC5; break;  // PHYSICAL XRAM READ
      case 0x50: type = 0xC3; break;  // PHYSICAL XRAM WRITE
      case 0x51: type = 0x6E; break;  // PHYSICAL PORT READ
      case 0x52: type = 0x6D; break;  // PHYSICAL PORT WRITE
      case 0x53: type = 0x9E; break;  // PHYSICAL BE READ
      case 0x54: type = 0x9D; break;  // PHYSICAL BE WRITE
      case 0x65: type = 0x33; break;  // PHYSICAL KMBUS RAM READ
      case 0x67: type = 0x43; break;  // PHYSICAL KMBUS EEPROM READ
      default:
        type_ok = false;
        break;
    }
  }

  if (!type_ok || type == 0x00) {
    ESP_LOGW(TAG,
             "GWG: discarding datapoint due to unknown type mapping: MSB=0x%02X addr=0x%02X full=0x%04X write=%d",
             func, addr, (unsigned) dp->address, (int) dp->write);
    _queue.pop();
    // delete dp; // only if this class owns dp allocation

    // Try next immediately.
    _state = SEND;
    return;
  }

  // Prepare the request parameters.
  // For EEPROM READ, the actual request is forced to length=1 and the address increments.
  uint8_t req_addr = addr;
  uint8_t req_len = logical_len;

  if (is_eeprom_read) {
    // Initialize the EEPROM multi-read tracker if not already active.
    // We keep dp in the queue until the final byte is assembled.
    if (!_eprom_read_active) {
      _eprom_read_active = true;
      _eprom_base_addr = addr;
      _eprom_total_len = logical_len;
      _eprom_index = 0;
      memset(_eprom_payload, 0, sizeof(_eprom_payload));

      // Defensive check: dp->length must fit our internal buffer.
      if (_eprom_total_len == 0 || _eprom_total_len > MAX_DP_LENGTH) {
        ESP_LOGW(TAG,
                 "EEPROM READ: invalid logical length %u (MAX_DP_LENGTH=%u), discarding dp full=0x%04X",
                 (unsigned) _eprom_total_len, (unsigned) MAX_DP_LENGTH, (unsigned) dp->address);
        _eprom_read_active = false;
        _queue.pop();
        // delete dp; // only if this class owns dp allocation
        _state = SEND;
        return;
      }
    }

    // For the current sub-read:
    // - request exactly 1 byte
    // - address increments by index
    req_len = 1;
    req_addr = (uint8_t) (_eprom_base_addr + _eprom_index);

    // If address would overflow 0xFF, abort cleanly.
    // (In practice, YAML should avoid cross-page reads, but this prevents undefined behavior.)
    if ((uint16_t) _eprom_base_addr + (uint16_t) _eprom_index > 0xFF) {
      ESP_LOGW(TAG,
               "EEPROM READ: address overflow base=0x%02X index=%u -> 0x%02X, discarding dp",
               _eprom_base_addr, (unsigned) _eprom_index, (unsigned) req_addr);
      _eprom_read_active = false;
      _queue.pop();
      // delete dp; // only if this class owns dp allocation
      _state = SEND;
      return;
    }
  }

  // Build and send frame.
  uint8_t buff[MAX_DP_LENGTH + 4];
  buff[0] = type;
  buff[1] = req_addr;
  buff[2] = req_len;
  buff[3] = 0x04;

  if (dp->write) {
    // WRITE: append payload
    memcpy(&buff[4], dp->data, dp->length);

    // Expected response is ACK (legacy behavior).
    // If some protocol variants differ, that should be handled per-type later.
    _rcvLen = 1;
    _uart->write_array(buff, 4 + dp->length);
  } else {
    // READ:
    // - For most reads: expected response equals requested length.
    // - For EEPROM READ: controller always returns 2 bytes (complement + payload),
    //   even though we request len=1.
    if (is_eeprom_read) {
      _rcvLen = 2;
    } else {
      _rcvLen = req_len;
    }
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
  ESP_LOGD(TAG, "READY->SEND delay: %lu ms", (unsigned long) (_sendMillis - _readyMillis));

  if (is_eeprom_read) {
    ESP_LOGD(TAG,
             "TX EEPROM READ: type=0x%02X base=0x%02X index=%u addr=0x%02X logical_len=%u sub_len=%u",
             type, _eprom_base_addr, (unsigned) _eprom_index, (unsigned) req_addr,
             (unsigned) _eprom_total_len, (unsigned) req_len);
  } else {
    ESP_LOGD(TAG, "TX: type=0x%02X func=0x%02X addr=0x%02X len=%u write=%d",
             type, func, addr, (unsigned) req_len, (int) dp->write);
  }

  _state = RECEIVE;
}

void OptolinkGWG::_receive() {
  // RECEIVE state:
  // Collect response bytes until expected response length is met or a timeout occurs.
  while (_uart->available() != 0) {
    uint8_t b = _uart->read();

    // Protect against buffer overflow.
    if (_rcvBufferLen >= sizeof(_rcvBuffer)) {
      ESP_LOGW(TAG, "RX buffer overflow (len=%d), resetting", (int) _rcvBufferLen);
      _rcvBufferLen = 0;
      memset(_rcvBuffer, 0, sizeof(_rcvBuffer));
      _state = INIT;
      _burstActive = false;

      // Abort any EEPROM multi-read as well.
      _eprom_read_active = false;
      _eprom_total_len = 0;
      _eprom_index = 0;

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

    OptolinkDP *dp = (_queue.size() > 0) ? _queue.front() : nullptr;
    const bool is_eeprom_read = _is_eeprom_read_(dp);

    // ------------------------------------------------------------------------
    // EEPROM READ special receive logic:
    // - Controller returns 2 bytes: [complement][payload]
    // - Verify complement rule: complement + payload == 0xFF
    // - Store payload byte into the assembled buffer
    // - If more bytes are needed, immediately SEND next sub-request
    // - Otherwise, call _tryOnData() once with the assembled payload buffer
    // ------------------------------------------------------------------------
    if (is_eeprom_read && _eprom_read_active) {
      const uint8_t b0 = _rcvBuffer[0];
      const uint8_t b1 = _rcvBuffer[1];

      // Validate complement rule.
      // The protocol observation: b0 completes b1 to 0xFF (b0 + b1 == 0xFF).
      // This sanity check prevents random bytes (e.g. late READY signals)
      // from being misinterpreted as valid EEPROM content.
      const uint8_t sum = (uint8_t) (b0 + b1);
      if (sum != 0xFF) {
      if (b0 == 0xFF && b1 == 0xFF) {
        ESP_LOGW(TAG,
                 "EEPROM READ: received 0xFF 0xFF, likely adress not in use, continueing");
      } else {
        ESP_LOGW(TAG,
                 "EEPROM READ: invalid complement pair: 0x%02X 0x%02X (sum=0x%02X), aborting dp",
                 b0, b1, sum);

        // Abort EEPROM read aggregation and reset.
        _eprom_read_active = false;
        _eprom_total_len = 0;
        _eprom_index = 0;

        // Treat this as a protocol error -> reset state machine.
        _state = INIT;
        _burstActive = false;
        _drain_uart_();
        _lastMillis = millis();
        return;
      }

      // Store the payload byte (second byte) into the assembled payload buffer.
      if (_eprom_index < _eprom_total_len) {
        _eprom_payload[_eprom_index] = b1;
      }

      ESP_LOGD(TAG,
               "RX EEPROM byte: base=0x%02X index=%u addr=0x%02X raw=[%02X %02X] payload=0x%02X time=%lu ms",
               _eprom_base_addr, (unsigned) _eprom_index,
               (unsigned) (uint8_t)(_eprom_base_addr + _eprom_index),
               b0, b1, b1, (unsigned long) rx_time);

      // Advance to next byte.
      _eprom_index++;

      // If more bytes are pending, immediately continue with SEND.
      // Burst mode is naturally satisfied here: we do NOT wait for READY (0x05),
      // and we do NOT send ACK (0x01), because ACK is only used to acknowledge READY.
      if (_eprom_index < _eprom_total_len) {
        _rcvBufferLen = 0;
        memset(_rcvBuffer, 0, sizeof(_rcvBuffer));

        _state = SEND;
        _lastMillis = millis();
        return;
      }

      // EEPROM aggregation completed:
      // Now forward the assembled payload buffer to the datapoint handler.
      // This is the only call to _tryOnData() for the whole multi-byte EEPROM read.
      ESP_LOGD(TAG,
               "RX EEPROM complete: base=0x%02X total_len=%u -> forwarding assembled payload",
               _eprom_base_addr, (unsigned) _eprom_total_len);

      _tryOnData(_eprom_payload, _eprom_total_len);

      // Reset EEPROM aggregation state AFTER forwarding.
      // The dp is expected to be popped/consumed by _tryOnData() as usual.
      _eprom_read_active = false;
      _eprom_total_len = 0;
      _eprom_index = 0;
      memset(_eprom_payload, 0, sizeof(_eprom_payload));

      _lastMillis = millis();

      // Continue burst mode as usual.
      if (_burstActive && _queue.size() > 0) {
        _state = SEND;
        return;
      }

      _burstActive = false;
      _state = IDLE;
      return;
    }

    // ------------------------------------------------------------------------
    // Normal (non-EEPROM) handling:
    // ------------------------------------------------------------------------
    if (dp != nullptr) {
      const uint8_t addr = dp->address & 0xFF;
      ESP_LOGD(TAG, "RX complete: addr=0x%02X len=%d time=%lu ms",
               (unsigned) addr, (int) _rcvBufferLen, (unsigned long) rx_time);
    } else {
      ESP_LOGD(TAG, "RX complete: len=%d time=%lu ms (no dp in queue?)",
               (int) _rcvBufferLen, (unsigned long) rx_time);
    }

    // Forward data to datapoint handler.
    // Note: Typically _tryOnData() will pop the datapoint from the queue
    // (depending on base implementation).
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
  if (_rcvBufferLen > 0 && millis() - _lastRxMillis > GWG_RX_INTERBYTE_TIMEOUT_MS) {
    ESP_LOGD(TAG, "Inter-byte timeout: got %d expected %d",
             (int) _rcvBufferLen, (int) _rcvLen);

    _rcvBufferLen = 0;
    memset(_rcvBuffer, 0, sizeof(_rcvBuffer));

    // Abort any EEPROM aggregation as well, because the frame broke.
    _eprom_read_active = false;
    _eprom_total_len = 0;
    _eprom_index = 0;

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
             (int) _rcvBufferLen, (int) _rcvLen,
             (unsigned long) (millis() - _sendMillis));

    _rcvBufferLen = 0;
    memset(_rcvBuffer, 0, sizeof(_rcvBuffer));

    // Abort any EEPROM aggregation as well, because we lost sync.
    _eprom_read_active = false;
    _eprom_total_len = 0;
    _eprom_index = 0;

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

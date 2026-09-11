// Copyright 2026 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// src/USBStream.cpp
#include "USBStream.h"

// tud_vendor_write_flush() is not exposed through Arduino's USBVendor wrapper
// (whose flush() is an empty function), so reach for the TinyUSB API directly.
#include "tusb.h"

// Push whatever is in the TX FIFO onto the wire now, rather than waiting for it
// to reach a full endpoint packet.
static inline void usbFlushTx() { tud_vendor_write_flush(); }

// ---------------------------------------------------------------------------
// Waiting for TX FIFO space, without busy-waiting.
//
// The FIFO holds one 64-byte packet, so any frame bigger than that waits for
// the first transfer to complete before the rest can be queued. Polling that
// with delayMicroseconds() burns roughly a millisecond of CPU per frame.
//
// tud_vendor_tx_cb() fires when a vendor IN transfer completes - precisely when
// space frees up. TinyUSB declares it TU_ATTR_WEAK and the Arduino core leaves
// it unimplemented, so we can own it and turn the wait into a sleep.
//
// The one-tick timeout on the wait, not the callback, is what guarantees
// progress: if the notification never arrives the loop simply retries, so
// nothing depends on the callback firing or on which context it runs in.
// ---------------------------------------------------------------------------
static volatile TaskHandle_t s_tx_waiter = nullptr;

extern "C" void tud_vendor_tx_cb(uint8_t itf, uint32_t sent_bytes) {
    (void)itf;
    (void)sent_bytes;
    TaskHandle_t waiter = s_tx_waiter;
    if (waiter == nullptr) return;

    if (xPortInIsrContext()) {
        BaseType_t higher_prio_woken = pdFALSE;
        vTaskNotifyGiveFromISR(waiter, &higher_prio_woken);
        if (higher_prio_woken) portYIELD_FROM_ISR();
    } else {
        xTaskNotifyGive(waiter);
    }
}

static inline void usbWaitTxSpace() { ulTaskNotifyTake(pdTRUE, 1); }

USBStream::USBStream()
    : _error(false)
    , _on_command(nullptr)
    , _field_count(0)
    , _bin_total(0)
{
    memset(_bin_buf, 0, sizeof(_bin_buf));
}

bool USBStream::add(const char* key, Type type, size_t count) {
    if (_field_count >= MAX_FIELDS) { _error = true; return false; }
    size_t required = _bin_total + _typeSize(type) * count;
    if (required > MAX_BUF)        { _error = true; return false; }

    FieldDef& f = _fields[_field_count++];
    f.key    = key;
    f.type   = type;
    f.count  = count;
    f.offset = _bin_total;
    _bin_total += _typeSize(type) * count;
    return true;
}

void USBStream::begin(uint16_t vid, uint16_t pid,
                      const char* manufacturer, const char* product)
{
    USB.VID(vid);
    USB.PID(pid);
    USB.manufacturerName(manufacturer);
    USB.productName(product);
    _vendor.begin();
    USB.begin();
}

bool USBStream::mounted() {
    return _vendor.mounted();
}

void USBStream::onCommand(CommandCallback cb) {
    _on_command = cb;
}

FieldDef* USBStream::_find(const char* key) {
    for (size_t i = 0; i < _field_count; i++) {
        if (strcmp(_fields[i].key, key) == 0) return &_fields[i];
    }
    return nullptr;
}

// set() - scalar
void USBStream::set(const char* key, uint32_t value) {
    FieldDef* f = _find(key); if (!f) return;
    memcpy(_bin_buf + f->offset, &value, sizeof(uint32_t));
}
void USBStream::set(const char* key, uint16_t value) {
    FieldDef* f = _find(key); if (!f) return;
    memcpy(_bin_buf + f->offset, &value, sizeof(uint16_t));
}
void USBStream::set(const char* key, uint8_t value) {
    FieldDef* f = _find(key); if (!f) return;
    _bin_buf[f->offset] = value;
}
void USBStream::set(const char* key, int32_t value) {
    FieldDef* f = _find(key); if (!f) return;
    memcpy(_bin_buf + f->offset, &value, sizeof(int32_t));
}
void USBStream::set(const char* key, int16_t value) {
    FieldDef* f = _find(key); if (!f) return;
    memcpy(_bin_buf + f->offset, &value, sizeof(int16_t));
}
void USBStream::set(const char* key, float value) {
    FieldDef* f = _find(key); if (!f) return;
    memcpy(_bin_buf + f->offset, &value, sizeof(float));
}
void USBStream::set(const char* key, bool value) {
    FieldDef* f = _find(key); if (!f) return;
    _bin_buf[f->offset] = value ? 1 : 0;
}

// set() - array
void USBStream::set(const char* key, const float* array, size_t len) {
    FieldDef* f = _find(key); if (!f) return;
    memcpy(_bin_buf + f->offset, array, sizeof(float) * len);
}
void USBStream::set(const char* key, const int16_t* array, size_t len) {
    FieldDef* f = _find(key); if (!f) return;
    memcpy(_bin_buf + f->offset, array, sizeof(int16_t) * len);
}
void USBStream::set(const char* key, const uint32_t* array, size_t len) {
    FieldDef* f = _find(key); if (!f) return;
    memcpy(_bin_buf + f->offset, array, sizeof(uint32_t) * len);
}
void USBStream::set(const char* key, const uint16_t* array, size_t len) {
    FieldDef* f = _find(key); if (!f) return;
    memcpy(_bin_buf + f->offset, array, sizeof(uint16_t) * len);
}
void USBStream::set(const char* key, const int32_t* array, size_t len) {
    FieldDef* f = _find(key); if (!f) return;
    memcpy(_bin_buf + f->offset, array, sizeof(int32_t) * len);
}
void USBStream::set(const char* key, const bool* array, size_t len) {
    FieldDef* f = _find(key); if (!f) return;
    for (size_t i = 0; i < len; i++) _bin_buf[f->offset + i] = array[i] ? 1 : 0;
}

// send()
uint8_t USBStream::_checksum() const {
    uint8_t cs = 0;
    for (size_t i = 0; i < _bin_total; i++) cs ^= _bin_buf[i];
    return cs;
}

bool USBStream::send() {
    if (_error || !mounted()) {
        _dropped_frames++;
        return false;
    }

    uint8_t out[MAX_BUF + 3];
    out[0] = PACKET_HEADER_0;
    out[1] = PACKET_HEADER_1;
    memcpy(out + 2, _bin_buf, _bin_total);
    out[_bin_total + 2] = _checksum();
    const size_t total = _bin_total + 3;

    size_t   sent    = 0;
    bool     stalled = false;
    const uint32_t t_start = millis();

    // Register before the first write so a completion cannot be missed.
    s_tx_waiter = xTaskGetCurrentTaskHandle();

    while (sent < total) {
        const size_t ret = _vendor.write(out + sent, min(USB_PACKET, total - sent));
        if (ret > 0) {
            sent += ret;
            // Flush every accepted chunk. The FIFO is one packet deep, so a
            // trailing partial packet would otherwise wait for the next frame
            // to push it out - and would never leave at all once streaming
            // stops, truncating the last frame the PC ever sees.
            usbFlushTx();
            continue;
        }

        stalled = true;
        if (millis() - t_start >= SEND_TIMEOUT_MS) {
            _stall_events++;
            _dropped_frames++;
            usbFlushTx();
            s_tx_waiter = nullptr;
            return false;
        }
        usbWaitTxSpace();
    }

    s_tx_waiter = nullptr;
    if (stalled) _stall_events++;
    _sent_frames++;
    return true;
}

// recv()
size_t USBStream::recv(uint8_t* buf, size_t len) {
    size_t received = _vendor.read(buf, len);
    if (received > 0 && _on_command != nullptr) {
        _on_command(buf, received);
    }
    return received;
}

// sendPingResponse()
void USBStream::sendPingResponse(const SensorSnapshot& snapshot,
                                 const char* fw, const char* hw,
                                 const char* updated) {
    if (!mounted()) return;

    // Layout is 2 + 16 + 16 + 12 + 1 + 18*fields + 5*NUM_SENSORS bytes, i.e.
    // 127 + 18*fields. The old 256-byte buffer overflowed at 8 fields with no
    // bounds check, so registering a few more stream fields would have
    // corrupted memory. Size it for the schema limit and check the fit.
    static uint8_t buf[64 + 18 * MAX_FIELDS + 5 * NUM_SENSORS];
    const size_t need = 47 + 18 * _field_count + 5 * NUM_SENSORS;
    if (need > sizeof(buf)) return;

    memset(buf, 0, sizeof(buf));
    size_t pos = 0;

    buf[pos++] = 0xA5;
    buf[pos++] = 0x50;

    strncpy((char*)buf + pos, fw,      16); pos += 16;
    strncpy((char*)buf + pos, hw,      16); pos += 16;
    strncpy((char*)buf + pos, updated, 12); pos += 12;

    buf[pos++] = (uint8_t)_field_count;
    for (size_t i = 0; i < _field_count; i++) {
        strncpy((char*)buf + pos, _fields[i].key, 16); pos += 16;
        buf[pos++] = (uint8_t)_fields[i].type;
        buf[pos++] = (uint8_t)_fields[i].count;
    }

    for (int i = 0; i < NUM_SENSORS; i++) {
        memcpy(buf + pos, &snapshot.sensors[i].angle, 4); pos += 4;
        buf[pos++] = snapshot.sensors[i].error ? 0 : 1;
    }

    size_t sent = 0;
    uint32_t t_start = millis();
    while (sent < pos) {
        size_t chunk = min(USB_PACKET, pos - sent);
        size_t ret = _vendor.write(buf + sent, chunk);
        if (ret > 0) {
            sent    += ret;
            t_start  = millis();
            usbFlushTx();
        } else {
            if (millis() - t_start > 100) break;
            delay(1);
        }
    }
    // The PC blocks on this reply during the handshake, so its tail must not be
    // left sitting in the FIFO.
    usbFlushTx();
}

// Utilities
size_t USBStream::_typeSize(Type type) {
    switch (type) {
        case Type::UINT32: return 4;
        case Type::UINT16: return 2;
        case Type::UINT8:  return 1;
        case Type::INT32:  return 4;
        case Type::INT16:  return 2;
        case Type::FLOAT:  return 4;
        case Type::BOOL:   return 1;
        default:           return 0;
    }
}

const char* USBStream::_typeName(Type type) {
    switch (type) {
        case Type::UINT32: return "UINT32";
        case Type::UINT16: return "UINT16";
        case Type::UINT8:  return "UINT8";
        case Type::INT32:  return "INT32";
        case Type::INT16:  return "INT16";
        case Type::FLOAT:  return "FLOAT";
        case Type::BOOL:   return "BOOL";
        default:           return "UNKNOWN";
    }
}

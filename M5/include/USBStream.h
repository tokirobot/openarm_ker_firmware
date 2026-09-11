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

// include/USBStream.h
#pragma once

#include <USB.h>
#include <USBVendor.h>
#include <Arduino.h>

#include "Common.h"
#include "Meta.h"

#ifndef STREAM_TYPES_DEFINED
#define STREAM_TYPES_DEFINED

enum class Type {
    UINT32,
    UINT16,
    UINT8,
    INT32,
    INT16,
    FLOAT,
    BOOL,
};

struct FieldDef {
    const char* key;
    Type        type;
    size_t      count;
    size_t      offset;
};

using CommandCallback = bool (*)(const uint8_t* buf, size_t len);
#endif

// ---------------------------------------------------------------------------
// USB transport notes (ESP32-S3 / TinyUSB vendor class)
//
// * The TX FIFO is CFG_TUD_VENDOR_TX_BUFSIZE = 64 bytes - exactly one
//   full-speed bulk packet - and comes from the prebuilt Arduino sdkconfig, so
//   it cannot be raised from build flags. A frame larger than 64 bytes
//   therefore always takes more than one transfer.
//
// * tud_vendor_write() only starts a transfer by itself once the FIFO holds a
//   whole endpoint packet, so a trailing partial packet sits there until the
//   *next* frame pushes it out - and is stranded for good when streaming stops.
//   It has to be flushed explicitly.
//
// * Arduino's USBVendor::flush() is an empty function; it is not the flush you
//   want. tud_vendor_write_flush() from tusb.h is.
//
// * USBVendor::write() silently clamps to tud_vendor_n_write_available(), so a
//   short write is normal and the caller must loop.
// ---------------------------------------------------------------------------
class USBStream {
public:
    static const size_t MAX_FIELDS = 32;
    static const size_t MAX_BUF    = 512;
    // One bulk packet on a full-speed device.
    static const size_t USB_PACKET = 64;
    // Give up on a frame rather than stalling behind a host that is not
    // reading. At 250 Hz a frame is 4 ms, so 5 ms drops the current frame
    // instead of delivering it stale.
    static const uint32_t SEND_TIMEOUT_MS = 5;

    USBStream();
    size_t getBinTotal() const { return _bin_total; }

    bool add(const char* key, Type type, size_t count = 1);

    void begin(uint16_t    vid          = USB_VID,
               uint16_t    pid          = USB_PID,
               const char* manufacturer = MANUFACTURER,
               const char* product      = PRODUCT);

    bool mounted();
    bool hasError() const { return _error; }

    // Link health. Without these a host that stops reading is indistinguishable
    // from one that is keeping up, because dropped frames were silent.
    uint32_t getSentFrames()    const { return _sent_frames; }
    uint32_t getDroppedFrames() const { return _dropped_frames; }
    uint32_t getStallEvents()   const { return _stall_events; }
    void     resetLinkCounters() { _sent_frames = _dropped_frames = _stall_events = 0; }

    void set(const char* key, uint32_t value);
    void set(const char* key, uint16_t value);
    void set(const char* key, uint8_t  value);
    void set(const char* key, int32_t  value);
    void set(const char* key, int16_t  value);
    void set(const char* key, float    value);
    void set(const char* key, bool     value);

    void set(const char* key, const uint32_t* array, size_t len);
    void set(const char* key, const uint16_t* array, size_t len);
    void set(const char* key, const int32_t*  array, size_t len);
    void set(const char* key, const int16_t*  array, size_t len);
    void set(const char* key, const float*    array, size_t len);
    void set(const char* key, const bool*     array, size_t len);

    bool   send();
    size_t recv(uint8_t* buf, size_t len);
    void   onCommand(CommandCallback cb);

    void sendPingResponse(const SensorSnapshot& snapshot,
                          const char* fw_version,
                          const char* hw_version,
                          const char* last_updated);

private:
    USBVendor       _vendor;
    bool            _error;
    CommandCallback _on_command;

    uint32_t _sent_frames    = 0;
    uint32_t _dropped_frames = 0;
    uint32_t _stall_events   = 0;

    FieldDef _fields[MAX_FIELDS];
    size_t   _field_count;
    size_t   _bin_total;
    uint8_t  _bin_buf[MAX_BUF];

    FieldDef*   _find(const char* key);
    uint8_t     _checksum() const;
    static size_t      _typeSize(Type type);
    static const char* _typeName(Type type);
};

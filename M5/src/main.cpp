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

#include <M5Unified.h>
#include <Preferences.h>
#include "Common.h"
#include "AngleProcessor.h"
#include "RSNexus.h"
#include "GUIHandler.h"
#include "M5UnitScroll.h"
#include "Meta.h"

#ifdef USE_USB
#include "USBStream.h"
USBStream stream;
#else
#include "SerialStream.h"
SerialStream stream(Serial);
#endif

// =====================================================
// Global objects
// =====================================================
RSNexus      rs485nexus(Serial2, RS485_EN_PIN);
GUIHandler   gui;
Preferences  preferences;
M5UnitScroll scroll;

// =====================================================
// Angle processors
// =====================================================
AngleProcessor angle_processors[NUM_SENSORS];

// =====================================================
// System state
// =====================================================
SystemState g_state;

// =====================================================
// Queues
// =====================================================
QueueHandle_t dataQueue;
QueueHandle_t guiQueue;

// =====================================================
// Shared encoder state
// =====================================================
bool encoder_found = false;
// volatile int16_t shared_encoder_value  = 0;  // unused
// volatile uint8_t shared_encoder_button = 0;  // unused



// =====================================================
// Stream control
//
// Starting the stream must clear the latch that stopped it, otherwise the very
// next acquisition frame re-triggers the same fault and drops straight back to
// STANDBY - which from the PC looks exactly like the start command being
// ignored. Both entry points go through here so they cannot drift apart.
// =====================================================
void requestStreamStart() {
    g_state.reset_jump_state = true;
    g_state.jump_detected    = false;
    g_state.mode             = AppMode::STREAM;
}

void requestStreamStop() {
    g_state.mode = AppMode::STANDBY;
}

void drawStopScreen() {
    M5.Display.fillScreen(BLACK);
    M5.Display.fillRoundRect(STOP_BTN_X, STOP_BTN_Y, STOP_BTN_W, STOP_BTN_H, 8, TFT_RED);
    M5.Display.setTextColor(TFT_WHITE);
    M5.Display.setTextDatum(middle_center);
    M5.Display.drawString("STOP", STOP_BTN_X + STOP_BTN_W / 2, STOP_BTN_Y + STOP_BTN_H / 2);
}

// =====================================================
// Core 0: Sensor task
// I2C peripheral polling task.
// To add a new sensor (e.g. lifter, IMU):
//   1. Initialize the device in this task before the loop
//   2. Read the device in the loop and store to shared variables
//   3. Add the shared variable to SensorSnapshot in Common.h
//   4. Set the value in acquisitionTask() snapshot
//   5. Register the field with stream.add() in setup()
// =====================================================
void sensorTask(void* pvParameters) {
    const uint32_t COLOR_IDLE  = 0x001900;
    const uint32_t COLOR_PRESS = 0x000019;

    if (scroll.begin(&Wire, SCROLL_ADDR, I2C_SDA_PIN, I2C_SCL_PIN, 100000U)) {
        encoder_found = true;
        scroll.resetEncoder();
        scroll.setLEDColor(COLOR_IDLE);
    }

    for (;;) {
        if (encoder_found) {
            // shared_encoder_value = scroll.getEncoderValue();  // unused

            bool btn = scroll.getButtonStatus();
            static bool prev_btn = false;
            if (btn != prev_btn) {
                prev_btn = btn;
                scroll.setLEDColor(btn ? COLOR_PRESS : COLOR_IDLE);
            }
            // shared_encoder_button = btn;  // unused
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// =====================================================
// Core 0: Acquisition task
// =====================================================
void acquisitionTask(void* pvParameters) {
    rs485nexus.begin(RS485_BAUD, RS485_RX, RS485_TX);

    SensorSnapshot snapshot;
    uint32_t last_request_time = millis();

    for (;;) {
        snapshot.timestamp = micros();

        rs485nexus.readPacket();
        uint32_t now = millis();

        if (now - last_request_time >= 1) {
            last_request_time += 1;
            rs485nexus.requestPacket(BULK); // Request all joints
        }

        for (int i = 0; i < NUM_SENSORS; i++) {
            float   raw_deg = rs485nexus.getAngleDeg(i + 1);
            bool    valid   = rs485nexus.isValid(i + 1);


            if (valid) {
                snapshot.sensors[i].raw_deg = raw_deg;  // Store raw degree for debugging/GUI
                float angle = angle_processors[i].process(raw_deg, g_state.jig_angle_offsets[i], true);

                // Apply joint-specific mechanical offset
                angle += ENCODER_CONFIG[i].mech_joint_offset;

                // Avoid +/-360 jump at initialization when sensor and expected position are on opposite sides of 0/360 boundary
                if(angle > ENCODER_CONFIG[i].mech_max + MARGIN_DEG){
                    angle -= 360.0f;
                }
                else if (angle < ENCODER_CONFIG[i].mech_min - MARGIN_DEG)
                {
                    angle += 360.0f;
                }

                snapshot.sensors[i].angle = angle;
                snapshot.sensors[i].error = false;

            } else {
                snapshot.sensors[i].angle = angle_processors[i].getCumulative();
                snapshot.sensors[i].error = true;
            }
        }

        // snapshot.encoder_value  = shared_encoder_value;   // unused
        // snapshot.encoder_button = shared_encoder_button;  // unused

        // Jump detection: distinguish glitch (transient) from zero calibration error (persistent)
        // Compares each frame to last_good_angle; if deviation persists for JUMP_CONFIRM_FRAMES,
        // it's a real zero error. If it recovers sooner, it was a glitch - substitute & continue.
        {
            static float   last_good_angle[NUM_SENSORS] = {};
            static bool    last_good_valid[NUM_SENSORS] = {};
            static uint8_t jump_count[NUM_SENSORS]      = {};

            if (g_state.reset_jump_state.exchange(false)) {
                memset(last_good_valid, 0, sizeof(last_good_valid));
                memset(jump_count,      0, sizeof(jump_count));
            }

            for (int i = 0; i < NUM_SENSORS; i++) {
                if (!snapshot.sensors[i].error) {
                    if (g_state.jump_detect_enabled &&
                        g_state.mode == AppMode::STREAM &&
                        last_good_valid[i] &&
                        !ENCODER_CONFIG[i].skip_jump_detect) {
                        float diff = fabsf(snapshot.sensors[i].angle - last_good_angle[i]);
                        if (diff >= JUMP_THRESHOLD_DEG) {
                            if (++jump_count[i] >= JUMP_CONFIRM_FRAMES) {
                                g_state.jump_detected = true;
                                g_state.mode = AppMode::STANDBY;
                            } else {
                                snapshot.sensors[i].angle = last_good_angle[i];
                            }
                        } else {
                            last_good_angle[i] = snapshot.sensors[i].angle;
                            jump_count[i] = 0;
                        }
                    } else {
                        last_good_angle[i] = snapshot.sensors[i].angle;
                        last_good_valid[i] = true;
                        jump_count[i] = 0;
                    }
                } else {
                    last_good_valid[i] = false;
                }
            }
        }

        // save jig zero offsets if requested from GUI
        if (g_state.zero_all) {
            g_state.zero_mask = (1u << NUM_SENSORS) - 1;
            g_state.zero_all  = false;
        }
        if (g_state.zero_mask > 0) {
            for (int i = 0; i < NUM_SENSORS; i++) {
                if ((g_state.zero_mask >> i) & 1) {
                    float save_val = snapshot.sensors[i].raw_deg;
                    if (ENCODER_CONFIG[i].invert)
                        save_val = 360.0f - fmod(save_val, 360.0f);
                    g_state.jig_angle_offsets[i] = save_val;
                    angle_processors[i].reset();
                }
            }
            preferences.begin("ker-cal", false);
            preferences.putBytes("offsets", g_state.jig_angle_offsets, sizeof(g_state.jig_angle_offsets));
            preferences.end();
            g_state.zero_mask = 0;
        }

        if (g_state.mode == AppMode::STREAM) {
            xQueueOverwrite(dataQueue, &snapshot);
        }
        xQueueOverwrite(guiQueue, &snapshot);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// =====================================================
// Core 1: GUI task
// STANDBY: Full GUI
// STREAM:  Bkack out with STOP button, minimal touch handling
// =====================================================
void guiTask(void* pvParameters) {
    static SensorSnapshot snapshot;
    static AppMode        last_mode = AppMode::STANDBY;

    for (;;) {
        M5.update();

        AppMode current_mode = g_state.mode;

        if (current_mode != last_mode) {
            if (current_mode == AppMode::STREAM) {
                M5.Display.fillScreen(BLACK);
                M5.Display.setTextColor(TFT_GREEN);
                M5.Display.setTextDatum(middle_center);
                M5.Display.setTextFont(&fonts::Font4);
                M5.Display.drawString("STREAMING START", 160, 120);
                vTaskDelay(pdMS_TO_TICKS(1000));

                drawStopScreen();
            } else if (current_mode == AppMode::STANDBY) {
                if (g_state.jump_detected) {
                    // We don't clear jump_detected here. Keep it until START is pressed.
                    M5.Display.fillScreen(BLACK);
                    M5.Display.setTextDatum(middle_center);
                    M5.Display.setFont(&fonts::Font4);
                    M5.Display.setTextColor(TFT_RED);
                    M5.Display.drawCentreString("Jump Detected", 160, 95);
                    M5.Display.setFont(&fonts::Font2);
                    M5.Display.setTextColor(TFT_WHITE);
                    M5.Display.drawCentreString("Recalibrate zero position", 160, 135);
                    vTaskDelay(pdMS_TO_TICKS(3000));
                }
                M5.Display.fillScreen(BLACK);
            }
            last_mode = current_mode;
        }

        if (current_mode == AppMode::STANDBY) {
            // Full GUI
            xQueuePeek(guiQueue, &snapshot, 0);
            gui.setJumpStopped(g_state.jump_detected);
            gui.setJumpDetectEnabled(g_state.jump_detect_enabled);
            GUICommand cmd = gui.tick(snapshot, AppMode::STANDBY);

            switch (cmd.type) {
                case GUICommand::Type::START:
                    requestStreamStart();
                    break;
                case GUICommand::Type::TOGGLE_JUMP_DETECT:
                    g_state.jump_detect_enabled = !g_state.jump_detect_enabled.load();
                    break;
                case GUICommand::Type::ZERO_ALL:
                    g_state.zero_all = true;
                    break;
                case GUICommand::Type::ZERO_MASK:
                    g_state.zero_mask = cmd.mask;
                    break;
                default:
                    break;
            }
            vTaskDelay(pdMS_TO_TICKS(33));  // 30fps

        } else {
            m5::touch_detail_t tp = M5.Touch.getDetail(0);
            if (tp.wasReleased()) {
                if (tp.x >= STOP_BTN_X && tp.x <= STOP_BTN_X + STOP_BTN_W &&
                    tp.y >= STOP_BTN_Y && tp.y <= STOP_BTN_Y + STOP_BTN_H) {
                    requestStreamStop();
                }
            }
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

// Defined below, after setup(), so it can sit next to loop() it replaced.
void streamTask(void* pvParameters);

// =====================================================
// Setup
// =====================================================
void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    M5.Speaker.end();

    // Load offsets
    preferences.begin("ker-cal", true);
    preferences.getBytes("offsets", g_state.jig_angle_offsets, sizeof(g_state.jig_angle_offsets));
    preferences.end();

    // GUI init
    gui.init();

    for (int i = 0; i < NUM_SENSORS; i++) {
        angle_processors[i] = AngleProcessor(
            ENCODER_CONFIG[i].invert,
            ENCODER_CONFIG[i].mech_min,
            ENCODER_CONFIG[i].mech_max
        );
    }

    // USB Stream schema
    // The PC reads this schema from the PING response and builds its unpack
    // format from it, so adding or retyping a field needs no PC-side change.
    stream.add("timestamp",      Type::UINT32);
    stream.add("angles",         Type::FLOAT, NUM_SENSORS);
    // One bit per channel instead of one byte: sixteen booleans cost sixteen
    // bytes on every frame to carry sixteen bits. Renamed rather than retyped
    // in place so a reader expecting the old list fails loudly instead of
    // silently treating an integer as one.
    stream.add("error_mask",     Type::UINT16);
    // Frame counter. Nothing else lets the PC tell "the M5 sent nothing" apart
    // from "the frame was lost on the way", which is the first thing to ask
    // whenever the stream looks out of sync.
    stream.add("seq",            Type::UINT32);
    // stream.add("encoder_value",  Type::INT16);  // unused
    // stream.add("encoder_button", Type::UINT8);  // unused

    stream.onCommand([](const uint8_t* buf, size_t len) -> bool {
        if (len == 0) return false;
        switch (buf[0]) {
            case 0x00:  // PING - a metadata query, which must not change the
                        // mode: the PC pings repeatedly while fetching the
                        // schema, so a health check would kill a live session.
                g_state.ping_requested = true;
                return true;
            case 0x01: requestStreamStop();  return true;
            case 0x02: requestStreamStart(); return true;
            case 0x03: g_state.zero_all = true;             return true;
            case 0x04:
                if (len >= 3)
                    g_state.zero_mask = (buf[1] << 8) | buf[2];
                return true;
            default: return false;
        }
    });

    #ifdef USE_USB
        stream.begin();
    #else
        Serial.begin(2000000);
    #endif

    // Start in STANDBY mode
    g_state.mode = AppMode::STANDBY;

    // Queues
    dataQueue = xQueueCreate(1, sizeof(SensorSnapshot));
    guiQueue  = xQueueCreate(1, sizeof(SensorSnapshot));

    delay(500);

    // Tasks
    xTaskCreatePinnedToCore(acquisitionTask, "Acquisition", 8192, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(sensorTask,      "Sensor",      4096, NULL, 2, NULL, 0);
    // Above guiTask: a full-screen canvas push holds the SPI bus for ~31 ms and
    // must not be able to hold up a stream frame.
    xTaskCreatePinnedToCore(streamTask,      "Stream",      4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(guiTask,         "GUI",         4096, NULL, 1, NULL, 1);
}

// =====================================================
// Core 1: Stream task
//
// This used to be loop(). Arduino's loopTask runs on core 1 at priority 1 -
// exactly where guiTask runs - and a full-screen M5Canvas push is
// 320*240*16 bit = 153600 bytes over a 40 MHz SPI bus, about 31 ms during which
// an equal-priority task gets no CPU. Streaming from loop() therefore stalled
// for most of every GUI frame.
//
// Priority 4 sits below acquisitionTask (5, and on core 0 anyway) and below the
// TinyUSB device task, but above guiTask and loopTask, so rendering can no
// longer delay a frame. It also blocks on the queue instead of polling, so a
// frame goes out as soon as acquisitionTask produces one.
// =====================================================
void streamTask(void* pvParameters) {
    static SensorSnapshot snapshot;
    uint8_t               cmd_buf[64];
    uint32_t              tx_seq = 0;

    for (;;) {
        stream.recv(cmd_buf, sizeof(cmd_buf));

        if (g_state.ping_requested.exchange(false)) {
            xQueuePeek(dataQueue, &snapshot, 0);
            stream.sendPingResponse(snapshot, FW_VERSION, HW_VERSION, LAST_UPDATED);
        }

        if (g_state.mode == AppMode::STANDBY) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (xQueueReceive(dataQueue, &snapshot, pdMS_TO_TICKS(2)) != pdTRUE) {
            continue;
        }

        float    angles[NUM_SENSORS];
        uint16_t error_mask = 0;
        for (int i = 0; i < NUM_SENSORS; i++) {
            angles[i] = snapshot.sensors[i].angle;
            if (snapshot.sensors[i].error) error_mask |= (uint16_t)(1u << i);
        }

        stream.set("timestamp",  snapshot.timestamp);
        stream.set("angles",     angles, NUM_SENSORS);
        stream.set("error_mask", error_mask);
        stream.set("seq",        tx_seq++);
        stream.send();
    }
}

// =====================================================
// Loop (Core 1)
//
// Streaming moved to streamTask so GUI rendering cannot delay it; nothing is
// left to do here.
// =====================================================
void loop() {
    vTaskDelay(pdMS_TO_TICKS(100));
}

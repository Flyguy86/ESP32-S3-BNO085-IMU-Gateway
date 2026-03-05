// =============================================================================
// serial_bridge.cpp — UART1 driver + ring-buffer pump
// =============================================================================
#include "serial_bridge.h"
#include "config_manager.h"
#include "pypilot_proto.h"
#include <HardwareSerial.h>
#include <driver/uart.h>
#include <driver/gpio.h>

SerialBridge serialBridge;                    // singleton

static HardwareSerial motorSerial(UART_NUM);

// ---------------------------------------------------------------------------
void SerialBridge::begin(uint32_t baud, int8_t txPin, int8_t rxPin) {
    // Configure RX pin with pull-up (idle-high for UART)
    pinMode(rxPin, INPUT_PULLUP);
    delay(10);

    motorSerial.begin(baud, SERIAL_8N1, rxPin, txPin);
    if (configManager.config().debugMode) {
        Serial.printf("[UART] Started UART%d  TX=GPIO%d  RX=GPIO%d  @ %lu baud\n",
                      UART_NUM, txPin, rxPin, baud);
        int rxLevel = digitalRead(rxPin);
        Serial.printf("[UART] RX pin GPIO%d idle level: %s\n",
                      rxPin, rxLevel ? "HIGH (normal)" : "LOW (check wiring!)");
    }
}

// ---------------------------------------------------------------------------
// Called at high frequency from the serial FreeRTOS task (Core 1).
// Drains the HW UART RX FIFO into the ring buffer so it never overflows.
// ---------------------------------------------------------------------------
void SerialBridge::poll() {
    bool dbg = configManager.config().debugMode;
    while (motorSerial.available()) {
        int b = motorSerial.read();
        if (b >= 0) {
            uint8_t byte = static_cast<uint8_t>(b);
            _rxBuf.push(byte);
            protoSniffer.feedByte(byte);   // passive decode for web console
            rawByteCount++;

            if (dbg) {
                // Store in debug buffer (circular overwrite)
                if (dbgLen < DBG_BUF_SIZE) {
                    dbgBuf[dbgLen++] = byte;
                }

                // Raw monitor buffer
                if (rawMonitorEnabled) {
                    size_t next = (monRxHead + 1) % MON_BUF_SIZE;
                    if (next != monRxTail) {
                        monRxBuf[monRxHead] = byte;
                        monRxHead = next;
                    }
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Read bytes out of the ring buffer (motor → host direction)
// ---------------------------------------------------------------------------
size_t SerialBridge::uartAvailable() {
    return _rxBuf.available();
}

size_t SerialBridge::uartRead(uint8_t* buf, size_t maxLen) {
    size_t n = 0;
    uint8_t b;
    while (n < maxLen && _rxBuf.pop(b)) {
        buf[n++] = b;
    }
    return n;
}

// ---------------------------------------------------------------------------
// Write bytes from host → motor via HW UART TX
// ---------------------------------------------------------------------------
size_t SerialBridge::uartWrite(const uint8_t* buf, size_t len) {
    size_t written = motorSerial.write(buf, len);
    txByteCount += written;

    // Raw monitor buffer for TX (only in debug mode)
    if (configManager.config().debugMode && rawMonitorEnabled) {
        for (size_t i = 0; i < written; i++) {
            size_t next = (monTxHead + 1) % MON_BUF_SIZE;
            if (next != monTxTail) {
                monTxBuf[monTxHead] = buf[i];
                monTxHead = next;
            }
        }
    }

    return written;
}

// ---------------------------------------------------------------------------
// Drain debug buffer into a hex string and reset
// ---------------------------------------------------------------------------
void SerialBridge::dbgDrain(char* hexOut, size_t maxChars) {
    size_t n = dbgLen;
    if (n == 0) { hexOut[0] = '\0'; return; }
    size_t pos = 0;
    for (size_t i = 0; i < n && (pos + 3) < maxChars; i++) {
        if (i > 0) hexOut[pos++] = ' ';
        snprintf(hexOut + pos, 3, "%02X", dbgBuf[i]);
        pos += 2;
    }
    hexOut[pos] = '\0';
    dbgLen = 0;  // reset
}

// ---------------------------------------------------------------------------
// Drain raw monitor buffers — returns count of RX bytes drained
// ---------------------------------------------------------------------------
size_t SerialBridge::monDrain(uint8_t* rxOut, size_t rxMax, size_t* rxCount,
                              uint8_t* txOut, size_t txMax, size_t* txCount) {
    size_t rxN = 0;
    while (rxN < rxMax && monRxTail != monRxHead) {
        rxOut[rxN++] = monRxBuf[monRxTail];
        monRxTail = (monRxTail + 1) % MON_BUF_SIZE;
    }
    size_t txN = 0;
    while (txN < txMax && monTxTail != monTxHead) {
        txOut[txN++] = monTxBuf[monTxTail];
        monTxTail = (monTxTail + 1) % MON_BUF_SIZE;
    }
    *rxCount = rxN;
    *txCount = txN;
    return rxN;
}

// ---------------------------------------------------------------------------
// FreeRTOS task — Core 1, high priority, tight loop
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// Re-init UART with different baud/pins (for baud scan diagnostics)
// ---------------------------------------------------------------------------
void SerialBridge::reinit(uint32_t baud, int8_t rxPin, int8_t txPin) {
    motorSerial.end();
    delay(10);
    pinMode(rxPin, INPUT_PULLUP);
    delay(5);
    motorSerial.begin(baud, SERIAL_8N1, rxPin, txPin);
}

// ---------------------------------------------------------------------------
// GPIO activity test — reads the pin rapidly and counts state changes.
// Detects if any electrical signal is present, independent of UART config.
// ---------------------------------------------------------------------------
uint32_t SerialBridge::gpioActivityTest(int pin, uint32_t durationMs) {
    // Temporarily detach UART from this pin to read it raw
    pinMode(pin, INPUT_PULLUP);
    delay(1);

    uint32_t transitions = 0;
    int lastState = digitalRead(pin);
    uint32_t start = millis();

    while ((millis() - start) < durationMs) {
        int state = digitalRead(pin);
        if (state != lastState) {
            transitions++;
            lastState = state;
        }
    }

    // Re-attach UART after test
    motorSerial.end();
    delay(10);
    motorSerial.begin(MOTOR_BAUD_DEFAULT, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

    return transitions;
}

// ---------------------------------------------------------------------------
// Passthrough task — simple bidirectional USB CDC ↔ UART1 forwarding.
// No protocol processing, no heartbeat. For direct PC→motor testing.
// ---------------------------------------------------------------------------
void passthroughSerialTask(void* /*param*/) {
    Serial.println("[PASSTHROUGH] USB CDC <-> UART1 passthrough active");
    Serial.printf("[PASSTHROUGH] UART TX=GPIO%d  RX=GPIO%d  @ %lu baud\n",
                  UART_TX_PIN, UART_RX_PIN, (unsigned long)MOTOR_BAUD_DEFAULT);
    Serial.println("[PASSTHROUGH] All USB serial data now forwarded to motor");
    delay(200);

    uint8_t buf[256];
    for (;;) {
        // USB CDC → UART (PC to motor)
        int avail = Serial.available();
        if (avail > 0) {
            int toRead = avail < (int)sizeof(buf) ? avail : (int)sizeof(buf);
            int n = Serial.readBytes(buf, toRead);
            if (n > 0) {
                serialBridge.uartWrite(buf, n);
            }
        }

        // UART → USB CDC (motor to PC)
        serialBridge.poll();
        size_t rxN = serialBridge.uartRead(buf, sizeof(buf));
        if (rxN > 0) {
            Serial.write(buf, rxN);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void serialTask(void* /*param*/) {
    // --- PyPilot motor controller initialization ---
    // The motor enters deep sleep after ~5s of no serial data.
    // Send 0xFF×4 to force it out of sync, then disengage to establish comms.
    delay(500);  // let UART settle

    // Step 1: Send 0xFF×4 reset/desync sequence (per pypilot arduino_servo.cpp)
    const uint8_t resetCode[] = {0xFF, 0xFF, 0xFF, 0xFF};
    serialBridge.uartWrite(resetCode, sizeof(resetCode));
    if (configManager.config().debugMode)
        Serial.println("[MOTOR] Sent 0xFF\xc3\x974 desync/wake sequence");
    delay(100);

    // Step 2: Send initial disengage commands to trigger SYNC
    for (int i = 0; i < 10; i++) {
        uint8_t pkt[4];
        ppBuildPacket(PP_DISENGAGE_CODE, 0, pkt);
        serialBridge.uartWrite(pkt, 4);
        delay(50);
        serialBridge.poll();  // check for response
    }
    if (configManager.config().debugMode)
        Serial.printf("[MOTOR] Init complete — RX bytes so far: %lu\n",
                      serialBridge.rawByteCount);

    // --- Main loop with heartbeat ---
    uint32_t lastHeartbeat = millis();
    // Debug mode: 400ms heartbeat (fast telemetry)
    // Normal mode: 2000ms heartbeat (power saving, motor sleeps at 5s)
    const uint32_t hbInterval = configManager.config().debugMode ? 400 : 2000;

    for (;;) {
        serialBridge.poll();

        // Heartbeat: send disengage periodically to prevent motor sleep
        // and keep telemetry flowing (motor only responds to commands)
        uint32_t now = millis();
        if ((now - lastHeartbeat) >= hbInterval) {
            uint8_t pkt[4];
            ppBuildPacket(PP_DISENGAGE_CODE, 0, pkt);
            serialBridge.uartWrite(pkt, 4);
            lastHeartbeat = now;
        }

        vTaskDelay(pdMS_TO_TICKS(1));   // ~1 kHz poll rate
    }
}

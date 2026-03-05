// =============================================================================
// serial_bridge.h — Hardware UART to ring-buffer abstraction
// =============================================================================
#pragma once

#include <cstdint>
#include <cstddef>
#include "config.h"

class SerialBridge {
public:
    void   begin(uint32_t baud = MOTOR_BAUD_DEFAULT,
                 int8_t txPin = UART_TX_PIN,
                 int8_t rxPin = UART_RX_PIN);

    // --- UART → wireless (motor → host) ------------------------------------
    size_t uartAvailable();                 // bytes waiting in RX ring buffer
    size_t uartRead(uint8_t* buf, size_t maxLen);

    // --- wireless → UART (host → motor) ------------------------------------
    size_t uartWrite(const uint8_t* buf, size_t len);

    // --- Internal pump — called from the serial FreeRTOS task ---------------
    void   poll();                          // move HW UART RX → ring buffer

    // --- Debug counters ----------------------------------------------------
    volatile uint32_t rawByteCount = 0;     // total raw bytes received
    volatile uint32_t txByteCount  = 0;     // total raw bytes transmitted

    // Debug hex buffer — last N raw bytes for display
    static const size_t DBG_BUF_SIZE = 64;
    uint8_t  dbgBuf[DBG_BUF_SIZE] = {};
    volatile size_t dbgLen = 0;             // bytes in dbgBuf (resets after read)
    void     dbgDrain(char* hexOut, size_t maxChars);  // drain & format as hex

    // Raw serial monitor — ring buffers for live streaming
    volatile bool rawMonitorEnabled = false;
    static const size_t MON_BUF_SIZE = 256;
    uint8_t  monRxBuf[MON_BUF_SIZE] = {};
    volatile size_t monRxHead = 0, monRxTail = 0;
    uint8_t  monTxBuf[MON_BUF_SIZE] = {};
    volatile size_t monTxHead = 0, monTxTail = 0;
    size_t   monDrain(uint8_t* rxOut, size_t rxMax, size_t* rxCount,
                      uint8_t* txOut, size_t txMax, size_t* txCount);

    // GPIO activity test — counts transitions on RX pin
    uint32_t gpioActivityTest(int pin, uint32_t durationMs);

    // Re-initialize UART with different settings (for baud scan)
    void reinit(uint32_t baud, int8_t rxPin, int8_t txPin);

private:
    // Simple lock-free single-producer/single-consumer ring buffer
    struct RingBuf {
        uint8_t  data[RING_BUF_SIZE];
        volatile size_t head = 0;
        volatile size_t tail = 0;

        size_t available() const {
            return (head - tail + RING_BUF_SIZE) % RING_BUF_SIZE;
        }
        bool push(uint8_t b) {
            size_t next = (head + 1) % RING_BUF_SIZE;
            if (next == tail) return false;   // full
            data[head] = b;
            head = next;
            return true;
        }
        bool pop(uint8_t& b) {
            if (head == tail) return false;    // empty
            b = data[tail];
            tail = (tail + 1) % RING_BUF_SIZE;
            return true;
        }
    };

    RingBuf _rxBuf;   // UART RX data (motor → host)
};

void serialTask(void* param);              // FreeRTOS task entry — runs on Core 1
void passthroughSerialTask(void* param);   // USB CDC ↔ UART1 passthrough task
extern SerialBridge serialBridge;

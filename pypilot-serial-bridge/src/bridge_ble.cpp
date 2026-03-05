// =============================================================================
// bridge_ble.cpp — BLE NUS (Nordic UART Service) transparent serial bridge
// =============================================================================
#include "bridge_ble.h"
#include "config.h"
#include "config_manager.h"
#include "serial_bridge.h"
#include "led_status.h"

#include <NimBLEDevice.h>

static NimBLEServer*         pServer       = nullptr;
static NimBLECharacteristic* pTxChar       = nullptr;   // notify (ESP→host)
static NimBLECharacteristic* pRxChar       = nullptr;   // write  (host→ESP)
static bool                  _bleConnected = false;
static uint16_t              _bleMTU       = 20;        // negotiated MTU payload

// ---------------------------------------------------------------------------
// Ring buffer for incoming BLE writes (host → motor)
// Single-producer (BLE callback), single-consumer (bridge loop)
// ---------------------------------------------------------------------------
static uint8_t   bleTxBuf[RING_BUF_SIZE];
static volatile size_t bleHead = 0, bleTail = 0;

static void blePush(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        size_t next = (bleHead + 1) % RING_BUF_SIZE;
        if (next == bleTail) break;          // full — drop
        bleTxBuf[bleHead] = data[i];
        bleHead = next;
    }
}

static size_t blePop(uint8_t* buf, size_t maxLen) {
    size_t n = 0;
    while (n < maxLen && bleHead != bleTail) {
        buf[n++] = bleTxBuf[bleTail];
        bleTail = (bleTail + 1) % RING_BUF_SIZE;
    }
    return n;
}

// ---------------------------------------------------------------------------
// BLE Server callbacks
// ---------------------------------------------------------------------------
class ServerCB : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* s, NimBLEConnInfo& ci) override {
        _bleConnected = true;
        _bleMTU = s->getPeerMTU(ci.getConnHandle()) - 3;  // ATT overhead
        if (_bleMTU < 20) _bleMTU = 20;
        Serial.printf("[BLE] Client connected  MTU payload=%u\n", _bleMTU);
        ledStatusSet(LedState::BLE_BRIDGING);
    }
    void onDisconnect(NimBLEServer* s, NimBLEConnInfo& ci, int reason) override {
        _bleConnected = false;
        Serial.printf("[BLE] Client disconnected (reason %d)\n", reason);
        ledStatusSet(LedState::BLE_ADVERTISING);
        NimBLEDevice::startAdvertising();     // restart advertising
    }
    void onMTUChange(uint16_t mtu, NimBLEConnInfo& ci) override {
        _bleMTU = mtu - 3;
        Serial.printf("[BLE] MTU changed → payload=%u\n", _bleMTU);
    }
};

// ---------------------------------------------------------------------------
// RX characteristic callback — host writes data here → UART
// ---------------------------------------------------------------------------
class RxCB : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& ci) override {
        const NimBLEAttValue& val = pChar->getValue();
        if (val.size() > 0) {
            blePush(val.data(), val.size());
        }
    }
};

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
void bridgeBleBegin() {
    const auto& cfg = configManager.config();

    // Build device name: "PyPilot-Bridge-XXXX"
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    char name[32];
    snprintf(name, sizeof(name), "%s-%02X%02X",
             BLE_DEVICE_PREFIX, mac[4], mac[5]);

    NimBLEDevice::init(name);
    NimBLEDevice::setMTU(247);                // request large MTU

    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCB());

    // ---- NUS service -------------------------------------------------------
    NimBLEService* pSvc = pServer->createService(NUS_SERVICE_UUID);

    // TX characteristic — notify (ESP → host)
    pTxChar = pSvc->createCharacteristic(
        NUS_TX_CHAR_UUID,
        NIMBLE_PROPERTY::NOTIFY
    );

    // RX characteristic — write (host → ESP)
    pRxChar = pSvc->createCharacteristic(
        NUS_RX_CHAR_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );
    pRxChar->setCallbacks(new RxCB());

    pSvc->start();

    // ---- Advertising -------------------------------------------------------
    NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
    pAdv->addServiceUUID(NUS_SERVICE_UUID);
    pAdv->setName(name);
    pAdv->start();

    Serial.printf("[BLE] NUS bridge advertising as '%s'\n", name);
    ledStatusSet(LedState::BLE_ADVERTISING);
}

bool bridgeBleClientConnected() {
    return _bleConnected;
}

// ---------------------------------------------------------------------------
// Main data-pump — called from bridgeBleTask
// ---------------------------------------------------------------------------
void bridgeBleLoop() {
    if (!_bleConnected) return;

    // ---- BLE RX → UART (host → motor) -------------------------------------
    {
        uint8_t buf[256];
        size_t n = blePop(buf, sizeof(buf));
        if (n > 0) {
            serialBridge.uartWrite(buf, n);
        }
    }

    // ---- UART → BLE TX (motor → host) -------------------------------------
    if (serialBridge.uartAvailable()) {
        uint8_t buf[240];
        size_t chunk = (_bleMTU < sizeof(buf)) ? _bleMTU : sizeof(buf);
        size_t n = serialBridge.uartRead(buf, chunk);
        if (n > 0) {
            pTxChar->setValue(buf, n);
            pTxChar->notify();
        }
    }
}

// ---------------------------------------------------------------------------
// FreeRTOS task — Core 0
// ---------------------------------------------------------------------------
void bridgeBleTask(void* /*param*/) {
    bridgeBleBegin();
    for (;;) {
        bridgeBleLoop();
        vTaskDelay(pdMS_TO_TICKS(5));       // BLE notify interval
    }
}

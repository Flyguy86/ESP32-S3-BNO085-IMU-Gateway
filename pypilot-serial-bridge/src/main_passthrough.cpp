// =============================================================================
// main_passthrough.cpp — Minimal USB-to-UART passthrough
// 
// Turns the ESP32-C3 into a simple USB-serial adapter.
// All bytes from USB CDC go to UART TX, all UART RX bytes go to USB CDC.
// No WiFi, no BLE, no web console. Just raw serial passthrough.
//
// Compile with: pio run -t upload
// Then use motor_test.py on the PC to talk to the motor controller.
// =============================================================================
#ifdef PASSTHROUGH_MODE

#include <Arduino.h>
#include <HardwareSerial.h>

#define TX_PIN  4
#define RX_PIN  5
#define BAUD    38400

HardwareSerial motorSerial(1);

void setup() {
    Serial.begin(115200); // USB CDC
    delay(2000);          // wait for USB to enumerate
    
    // Configure RX pin
    pinMode(RX_PIN, INPUT_PULLUP);
    delay(10);
    
    motorSerial.begin(BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
    
    Serial.println("=== USB-UART PASSTHROUGH MODE ===");
    Serial.printf("TX=GPIO%d  RX=GPIO%d  @ %d baud\n", TX_PIN, RX_PIN, BAUD);
    Serial.printf("RX idle: %s\n", digitalRead(RX_PIN) ? "HIGH (ok)" : "LOW (check wiring)");
    Serial.println("Forwarding all bytes bidirectionally...");
    Serial.println("Send 'T' for TX pin swap test, 'B' for baud cycle");
}

// Track pin orientation
bool swapped = false;
int currentBaud = BAUD;
const int bauds[] = {9600, 19200, 38400, 57600, 115200};
int baudIdx = 2; // start at 38400

void reinitUart() {
    motorSerial.end();
    delay(10);
    int tx = swapped ? RX_PIN : TX_PIN;
    int rx = swapped ? TX_PIN : RX_PIN;
    pinMode(rx, INPUT_PULLUP);
    delay(5);
    motorSerial.begin(currentBaud, SERIAL_8N1, rx, tx);
    Serial.printf("\n[REINIT] TX=GPIO%d RX=GPIO%d @ %d baud (RX idle=%s)\n",
                  tx, rx, currentBaud,
                  digitalRead(rx) ? "HIGH" : "LOW");
}

void loop() {
    // USB → UART (with command intercept)
    while (Serial.available()) {
        int c = Serial.read();
        
        // Special commands (uppercase single char)
        if (c == 'T') {
            swapped = !swapped;
            Serial.printf("\n[SWAP] Pins %s\n", swapped ? "SWAPPED" : "NORMAL");
            reinitUart();
            continue;
        }
        if (c == 'B') {
            baudIdx = (baudIdx + 1) % 5;
            currentBaud = bauds[baudIdx];
            Serial.printf("\n[BAUD] Switching to %d\n", currentBaud);
            reinitUart();
            continue;
        }
        
        motorSerial.write(c);
    }
    
    // UART → USB  
    while (motorSerial.available()) {
        int c = motorSerial.read();
        Serial.write(c);
    }
}

#endif // PASSTHROUGH_MODE

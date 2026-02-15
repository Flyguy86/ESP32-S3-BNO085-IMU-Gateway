/*
 * Project: High-Precision BNO085 IMU Gateway for ESP32-S3
 * Hardware: ESP32-S3-N16R8 + BNO085 (9DOF)
 * Wiring: BNO SDA -> GPIO 1, BNO SCL -> GPIO 2 (I2C)
 *
 * LED Status Codes (built-in RGB NeoPixel):
 *   Flashing BLUE    = WiFi connecting / captive portal
 *   Flashing YELLOW  = BNO085 IMU initializing
 *   Flashing RED     = BNO085 not detected (error)
 *   Flashing CYAN    = BLE advertising, waiting for connections
 *   Solid GREEN      = All systems running normally
 *   Pulsing GREEN    = Running + transmitting data
 */

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <DNSServer.h>
#include <ESPAsyncWiFiManager.h>
#include <WiFiUdp.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <SparkFun_BNO080_Arduino_Library.h>

// I2C Pin Mapping for "Sandwich" Solder
#define SDA_PIN 8
#define SCL_PIN 9

// Built-in RGB LED (NeoPixel on GPIO 48)
#define RGB_LED_PIN 48
#define RGB_BRIGHTNESS 10  // 0-255, keep low to avoid blinding

// LED helper: set the built-in RGB LED color
void setLED(uint8_t r, uint8_t g, uint8_t b) {
  neopixelWrite(RGB_LED_PIN, r, g, b);
}

// LED helper: flash a color n times at given interval (blocking)
void flashLED(uint8_t r, uint8_t g, uint8_t b, int times, int ms) {
  for (int i = 0; i < times; i++) {
    setLED(r, g, b);
    delay(ms);
    setLED(0, 0, 0);
    delay(ms);
  }
}

// Global Objects
BNO080 myIMU;
WiFiUDP udp;
AsyncWebServer server(80);
AsyncEventSource events("/events");
SemaphoreHandle_t radioMutex;

// BLE Objects
BLECharacteristic *pCharacteristic;
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

struct IMUData {
  float h, p, r;          // Heading, Pitch, Roll (degrees)
  float rot;               // Rate of turn - gyro Z (deg/s)
  float gx, gy, gz;        // Gyro X/Y/Z (deg/s)
  float lax, lay, laz;     // Linear acceleration X/Y/Z (m/s²)
  byte quatAcc;            // Quaternion accuracy (0-3)
  byte magAcc;             // Magnetometer accuracy (0-3)
} currentData;

// --- CORE 1: SENSOR TASK (High Priority) ---
void sensorTask(void * pvParameters) {
  // Enable rotation vector (uses mag for absolute heading) at 2Hz
  myIMU.enableRotationVector(100);
  delay(50);
  // Enable gyro for rate of turn (critical for autopilot PID)
  myIMU.enableGyro(100);
  delay(50);
  // Enable linear accelerometer (motion without gravity)
  myIMU.enableLinearAccelerometer(100);
  delay(50);
  // Enable magnetometer for mag calibration tracking
  myIMU.enableMagnetometer(500);
  delay(50);
  // Tell BNO085 to calibrate all sensors (accel, gyro, mag)
  myIMU.calibrateAll();

  for(;;) {
    while (myIMU.dataAvailable()) {
      float rawYaw = myIMU.getYaw() * 180.0 / PI;
      
      // Orientation
      currentData.h = (rawYaw < 0) ? rawYaw + 360.0 : rawYaw;
      currentData.p = myIMU.getPitch() * 180.0 / PI;
      currentData.r = myIMU.getRoll() * 180.0 / PI;

      // Gyro (deg/s) - rate of turn
      currentData.gx = myIMU.getGyroX() * 180.0 / PI;
      currentData.gy = myIMU.getGyroY() * 180.0 / PI;
      currentData.gz = myIMU.getGyroZ() * 180.0 / PI;
      currentData.rot = currentData.gz; // Z-axis = rate of turn for boats

      // Linear acceleration (m/s², gravity removed)
      currentData.lax = myIMU.getLinAccelX();
      currentData.lay = myIMU.getLinAccelY();
      currentData.laz = myIMU.getLinAccelZ();

      // Accuracy
      currentData.quatAcc = myIMU.getQuatAccuracy();
      currentData.magAcc = myIMU.getMagAccuracy();
    } // end while - drain all pending reports
    vTaskDelay(pdMS_TO_TICKS(10)); 
  }
}

// --- CORE 0: COMMS TASK (WiFi/BLE) ---
void commsTask(void * pvParameters) {
  for(;;) {
    bool wifiActive = (WiFi.status() == WL_CONNECTED);

    if (xSemaphoreTake(radioMutex, portMAX_DELAY)) {
      String payload = "{\"h\":" + String(currentData.h, 1) + 
                       ",\"p\":" + String(currentData.p, 1) + 
                       ",\"r\":" + String(currentData.r, 1) + 
                       ",\"rot\":" + String(currentData.rot, 2) + 
                       ",\"gx\":" + String(currentData.gx, 2) + 
                       ",\"gy\":" + String(currentData.gy, 2) + 
                       ",\"gz\":" + String(currentData.gz, 2) + 
                       ",\"lax\":" + String(currentData.lax, 3) + 
                       ",\"lay\":" + String(currentData.lay, 3) + 
                       ",\"laz\":" + String(currentData.laz, 3) +
                       ",\"qacc\":" + String(currentData.quatAcc) + 
                       ",\"macc\":" + String(currentData.magAcc) + "}";

      if (wifiActive) {
        // WiFi UDP Broadcast
        udp.beginPacket("255.255.255.255", 4210);
        udp.print(payload);
        udp.endPacket();
        events.send(payload.c_str(), "message", millis());
      }

      // BLE Always Broadcasts (Failover)
      pCharacteristic->setValue(payload.c_str());
      pCharacteristic->notify();

      xSemaphoreGive(radioMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(500)); // Maintain 2Hz Output
  }
}

// --- LED PULSE TASK (runs independently) ---
void ledTask(void * pvParameters) {
  for(;;) {
    // Slow green pulse: 3s fade up, 5s fade down
    for (int i = 0; i <= RGB_BRIGHTNESS; i++) {
      setLED(0, i, 0);
      vTaskDelay(pdMS_TO_TICKS(3000 / (RGB_BRIGHTNESS + 1)));
    }
    for (int i = RGB_BRIGHTNESS; i >= 0; i--) {
      setLED(0, i, 0);
      vTaskDelay(pdMS_TO_TICKS(5000 / (RGB_BRIGHTNESS + 1)));
    }
  }
}

void setup() {
  Serial.begin(115200);
  
  // Boot indicator - brief white flash
  setLED(RGB_BRIGHTNESS, RGB_BRIGHTNESS, RGB_BRIGHTNESS);
  delay(300);
  setLED(0, 0, 0);

  // Initialize I2C with Custom Pins
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // --- WiFi: Optional, non-blocking ---
  Serial.println("Trying WiFi (5s timeout)...");
  setLED(0, 0, RGB_BRIGHTNESS); // Blue while trying
  WiFi.mode(WIFI_STA);
  WiFi.begin(); // Use saved credentials
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 5000) {
    flashLED(0, 0, RGB_BRIGHTNESS, 1, 250);
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected! IP: ");
    Serial.println(WiFi.localIP());
    flashLED(0, RGB_BRIGHTNESS, 0, 3, 150);
  } else {
    Serial.println("WiFi not available - continuing with BLE only");
    flashLED(RGB_BRIGHTNESS, RGB_BRIGHTNESS/2, 0, 2, 200); // Yellow = no WiFi
  }

  // --- IMU: Flashing YELLOW while trying ---
  Serial.println("Initializing BNO085...");
  setLED(RGB_BRIGHTNESS, RGB_BRIGHTNESS/2, 0); // Yellow
  if (!myIMU.begin()) {
    Serial.println("BNO085 not detected. Check SDA=GPIO8, SCL=GPIO9!");
    // Flashing RED = error, won't proceed
    while(1) {
      flashLED(RGB_BRIGHTNESS, 0, 0, 1, 500);
    }
  }
  Serial.println("BNO085 detected!");
  flashLED(RGB_BRIGHTNESS, RGB_BRIGHTNESS/2, 0, 2, 150); // Yellow confirm

  // --- BLE: Flashing CYAN ---
  Serial.println("Starting BLE...");
  setLED(0, RGB_BRIGHTNESS, RGB_BRIGHTNESS); // Cyan
  BLEDevice::init("S3_IMU_GATEWAY");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("BLE advertising as S3_IMU_GATEWAY");
  flashLED(0, RGB_BRIGHTNESS, RGB_BRIGHTNESS, 3, 150); // Cyan confirm

  // Web Dashboard
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = "<html><body style='background:#222; color:#0f0; text-align:center;'>";
    html += "<h1>BNO085 S3 Gateway</h1><h2 id='h'>0.0</h2>";
    html += "<p>Accuracy: <span id='a'>0</span>/3</p><button onclick=\"fetch('/save')\">SAVE CALIBRATION</button>";
    html += "<script>var s=new EventSource('/events');s.onmessage=function(e){";
    html += "var d=JSON.parse(e.data);document.getElementById('h').innerHTML=d.h+'&deg;';";
    html += "document.getElementById('a').innerHTML=d.acc;};</script></body></html>";
    request->send(200, "text/html", html);
  });

  server.on("/save", HTTP_GET, [](AsyncWebServerRequest *request){
    myIMU.saveCalibration();
    request->send(200, "text/plain", "Flash Updated");
  });

  server.addHandler(&events);
  server.begin();

  radioMutex = xSemaphoreCreateMutex();

  // All systems go - solid GREEN
  setLED(0, RGB_BRIGHTNESS, 0);
  Serial.println("All systems running!");

  // Dual Core Tasking
  xTaskCreatePinnedToCore(sensorTask, "Sensor", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(commsTask, "Comms", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(ledTask, "LED", 2048, NULL, 0, NULL, 0);
}

void loop() { /* Logic handled in tasks */ }
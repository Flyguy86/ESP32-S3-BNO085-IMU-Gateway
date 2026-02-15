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
#include <ESPmDNS.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <SparkFun_BNO080_Arduino_Library.h>
#include <Preferences.h>

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
DNSServer dns;
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

// Signal K delta JSON UDP port (configurable via web UI, stored in NVS)
// In Signal K server: add connection -> "Signal K (Delta) over UDP" on this port
#define SIGNALK_UDP_PORT_DEFAULT 10110
uint16_t signalkUdpPort = SIGNALK_UDP_PORT_DEFAULT;
Preferences prefs;

// Send Signal K delta JSON via UDP broadcast
// SK natively parses this format — no NMEA sentence limitations
void sendSignalKDelta() {
  float headRad  = currentData.h   * DEG_TO_RAD;
  float pitchRad = currentData.p   * DEG_TO_RAD;
  float rollRad  = currentData.r   * DEG_TO_RAD;
  float yawRad   = headRad;  // yaw = heading for marine convention
  float rotRad   = currentData.rot * DEG_TO_RAD; // deg/s -> rad/s

  // Accuracy: 0=unreliable, 1=low, 2=medium, 3=high
  // Normalize to 0.0-1.0 ratio for Signal K convention
  float magAccNorm  = currentData.magAcc  / 3.0f;
  float quatAccNorm = currentData.quatAcc / 3.0f;

  char delta[600];
  snprintf(delta, sizeof(delta),
    "{\"updates\":[{\"source\":{\"label\":\"BNO085\",\"type\":\"IMU\"},"
    "\"values\":["
    "{\"path\":\"navigation.headingMagnetic\",\"value\":%.6f},"
    "{\"path\":\"navigation.rateOfTurn\",\"value\":%.6f},"
    "{\"path\":\"navigation.attitude\",\"value\":{\"roll\":%.6f,\"pitch\":%.6f,\"yaw\":%.6f}},"
    "{\"path\":\"sensors.imu.compassCalibration\",\"value\":%.2f},"
    "{\"path\":\"sensors.imu.fusionCalibration\",\"value\":%.2f}"
    "]}]}",
    headRad, rotRad, rollRad, pitchRad, yawRad, magAccNorm, quatAccNorm);

  udp.beginPacket("255.255.255.255", signalkUdpPort);
  udp.print(delta);
  udp.endPacket();
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
        // JSON UDP Broadcast (port 4210)
        udp.beginPacket("255.255.255.255", 4210);
        udp.print(payload);
        udp.endPacket();
        events.send(payload.c_str(), "message", millis());

        // Signal K delta JSON broadcast (all data in SI units)
        sendSignalKDelta();
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
  delay(2000); // Allow USB-Serial/JTAG to enumerate
  Serial.println("\n=== S3_IMU_GATEWAY Boot ===");
  
  // Boot indicator - brief white flash
  setLED(RGB_BRIGHTNESS, RGB_BRIGHTNESS, RGB_BRIGHTNESS);
  delay(300);
  setLED(0, 0, 0);

  // Load saved settings from NVS
  prefs.begin("config", false);
  signalkUdpPort = prefs.getUShort("skport", SIGNALK_UDP_PORT_DEFAULT);
  Serial.printf("Signal K UDP port: %d\n", signalkUdpPort);

  // Initialize I2C with Custom Pins
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // --- WiFi: Captive Portal via ESPAsyncWiFiManager ---
  Serial.println("Starting WiFiManager...");
  setLED(0, 0, RGB_BRIGHTNESS); // Blue while configuring

  AsyncWiFiManager wifiManager(&server, &dns);
  wifiManager.setAPCallback([](AsyncWiFiManager *mgr) {
    Serial.println("WiFi captive portal active!");
    Serial.print("Connect to AP: ");
    Serial.println(mgr->getConfigPortalSSID());
    Serial.println("Then open http://192.168.4.1");
  });
  // Portal stays open for 3 minutes, then continues with BLE-only
  wifiManager.setConfigPortalTimeout(180);

  // autoConnect: tries saved creds first, if fail -> opens AP "S3_IMU_GATEWAY"
  if (wifiManager.autoConnect("S3_IMU_GATEWAY")) {
    Serial.print("WiFi connected! IP: ");
    Serial.println(WiFi.localIP());
    if (MDNS.begin("s3imu")) {
      Serial.println("mDNS: http://s3imu.local");
      MDNS.addService("http", "tcp", 80);
    }
    flashLED(0, RGB_BRIGHTNESS, 0, 3, 150); // Green flash = connected
  } else {
    Serial.println("WiFi portal timed out - continuing with BLE only");
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

  // Web Dashboard (set up after WiFiManager releases the server)
  server.reset(); // Clear WiFiManager routes
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>";
    html += "<style>body{background:#222;color:#0f0;text-align:center;font-family:monospace;margin:20px}";
    html += "h1{font-size:1.4em}h2{font-size:3em;margin:10px}";
    html += "input,button{font-size:1em;padding:8px 16px;margin:4px;border-radius:6px;border:1px solid #0f0}";
    html += "input{background:#333;color:#0f0;width:80px;text-align:center}";
    html += "button{background:#0f0;color:#222;cursor:pointer}button:hover{background:#0a0}";
    html += ".info{color:#888;font-size:0.85em}.section{margin:20px auto;padding:10px;border:1px solid #333;border-radius:8px;max-width:400px}";
    html += "</style></head><body>";
    html += "<h1>BNO085 S3 Gateway</h1>";
    html += "<h2 id='h'>---</h2>";
    html += "<p>Pitch: <span id='p'>--</span>&deg; &nbsp; Roll: <span id='r'>--</span>&deg;</p>";
    html += "<p>ROT: <span id='rot'>--</span>&deg;/s</p>";
    html += "<p>Quat Accuracy: <span id='qa'>0</span>/3 &nbsp; Mag Accuracy: <span id='ma'>0</span>/3</p>";
    html += "<div class='section'><b>Signal K UDP Port</b><br>";
    html += "<input id='port' type='number' min='1024' max='65535' value='" + String(signalkUdpPort) + "'>";
    html += "<button onclick=\"fetch('/setport?port='+document.getElementById('port').value)";
    html += ".then(r=>r.text()).then(t=>{document.getElementById('ps').innerHTML=t;setTimeout(()=>document.getElementById('ps').innerHTML='',3000)})\">SET</button>";
    html += "<p id='ps' class='info'></p>";
    html += "<p class='info'>Current: port <span id='cp'>" + String(signalkUdpPort) + "</span> (UDP broadcast)</p></div>";
    html += "<div class='section'>";
    html += "<button onclick=\"fetch('/save').then(r=>r.text()).then(t=>alert(t))\">SAVE CALIBRATION</button> ";
    html += "<button onclick=\"if(confirm('Clear WiFi credentials and reboot?'))fetch('/resetwifi')\">RESET WIFI</button>";
    html += "</div>";
    html += "<script>var s=new EventSource('/events');s.onmessage=function(e){";
    html += "var d=JSON.parse(e.data);";
    html += "document.getElementById('h').innerHTML=d.h.toFixed(1)+'&deg;';";
    html += "document.getElementById('p').innerHTML=d.p.toFixed(1);";
    html += "document.getElementById('r').innerHTML=d.r.toFixed(1);";
    html += "document.getElementById('rot').innerHTML=d.rot.toFixed(2);";
    html += "document.getElementById('qa').innerHTML=d.qacc;";
    html += "document.getElementById('ma').innerHTML=d.macc;";
    html += "};</script></body></html>";
    request->send(200, "text/html", html);
  });

  server.on("/setport", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("port")) {
      uint16_t newPort = request->getParam("port")->value().toInt();
      if (newPort >= 1024 && newPort <= 65535) {
        signalkUdpPort = newPort;
        prefs.putUShort("skport", signalkUdpPort);
        Serial.printf("Signal K UDP port changed to %d\n", signalkUdpPort);
        request->send(200, "text/plain", "Port set to " + String(signalkUdpPort) + " (saved)");
      } else {
        request->send(400, "text/plain", "Invalid port (1024-65535)");
      }
    } else {
      request->send(400, "text/plain", "Missing ?port= parameter");
    }
  });

  server.on("/save", HTTP_GET, [](AsyncWebServerRequest *request){
    myIMU.saveCalibration();
    request->send(200, "text/plain", "Flash Updated");
  });

  server.on("/resetwifi", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "WiFi credentials cleared. Rebooting...");
    delay(1000);
    WiFi.disconnect(true, true); // Erase saved credentials
    ESP.restart();
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
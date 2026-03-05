// =============================================================================
// led_status.cpp — Non-blocking NeoPixel animations driven by state machine
// =============================================================================
#include "led_status.h"
#include "config.h"
#include <Adafruit_NeoPixel.h>

static Adafruit_NeoPixel pixel(RGB_LED_COUNT, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);
static volatile LedState currentState = LedState::OFF;

// ---------------------------------------------------------------------------
// Colour helpers  (GRB order handled by library)
// ---------------------------------------------------------------------------
static uint32_t colRed()    { return pixel.Color(255,   0,   0); }
static uint32_t colGreen()  { return pixel.Color(  0, 255,   0); }
static uint32_t colBlue()   { return pixel.Color(  0,   0, 255); }
static uint32_t colYellow() { return pixel.Color(255, 180,   0); }
static uint32_t colCyan()   { return pixel.Color(  0, 200, 200); }
static uint32_t colOff()    { return pixel.Color(  0,   0,   0); }

// Scale a colour by brightness 0-255
static uint32_t scale(uint32_t c, uint8_t b) {
    uint8_t r = ((c >> 16) & 0xFF) * b / 255;
    uint8_t g = ((c >>  8) & 0xFF) * b / 255;
    uint8_t bl = (c        & 0xFF) * b / 255;
    return pixel.Color(r, g, bl);
}

// Triangle-wave brightness (period ms)
static uint8_t breathe(uint32_t periodMs) {
    uint32_t phase = millis() % periodMs;
    uint32_t half  = periodMs / 2;
    if (phase < half) return (uint8_t)(phase * 255 / half);
    else              return (uint8_t)((periodMs - phase) * 255 / half);
}

// ---------------------------------------------------------------------------
void ledStatusBegin() {
    pixel.begin();
    pixel.setBrightness(40);   // keep it subtle
    pixel.show();
}

void ledStatusSet(LedState s) {
    currentState = s;
}

// ---------------------------------------------------------------------------
// FreeRTOS task — runs on Core 0 at lowest priority
// ---------------------------------------------------------------------------
void ledStatusTask(void* /*param*/) {
    for (;;) {
        uint32_t c = colOff();

        switch (currentState) {
            case LedState::OFF:
                c = colOff();
                break;
            case LedState::BOOT:
                c = scale(colRed(), breathe(1000));
                break;
            case LedState::SETUP:
                c = scale(colBlue(), breathe(2000));
                break;
            case LedState::WIFI_WAITING:
                c = scale(colYellow(), breathe(2500));
                break;
            case LedState::WIFI_BRIDGING:
                c = colGreen();
                break;
            case LedState::BLE_ADVERTISING:
                c = scale(colCyan(), breathe(2500));
                break;
            case LedState::BLE_BRIDGING:
                c = colCyan();
                break;
            case LedState::ERROR:
                c = (millis() / 200) & 1 ? colRed() : colOff();
                break;
        }

        pixel.setPixelColor(0, c);
        pixel.show();
        vTaskDelay(pdMS_TO_TICKS(20));   // ~50 fps
    }
}

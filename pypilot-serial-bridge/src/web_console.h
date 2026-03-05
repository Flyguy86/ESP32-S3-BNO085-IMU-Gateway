// =============================================================================
// web_console.h — Diagnostic web console with WebSocket (real-time telemetry
//                  + manual serial command buttons)
// =============================================================================
#pragma once

#include <ESPAsyncWebServer.h>

/// Attach the /console page and /ws WebSocket to an existing AsyncWebServer.
/// Call once after the server is created but before server.begin().
void webConsoleAttach(AsyncWebServer& server);

/// Call from the bridge loop at ~10 Hz to push telemetry JSON to all
/// connected WebSocket clients.
void webConsolePushTelemetry();

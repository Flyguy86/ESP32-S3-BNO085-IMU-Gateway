// =============================================================================
// bridge_wifi.h — WiFi STA + TCP server + mDNS/DNS-SD bridge
// =============================================================================
#pragma once

void bridgeWifiBegin();                // connect STA, start TCP server + mDNS
void bridgeWifiLoop();                 // handle client data (called from task)
void bridgeWifiTask(void* param);      // FreeRTOS task entry
bool bridgeWifiClientConnected();      // is a TCP client active?

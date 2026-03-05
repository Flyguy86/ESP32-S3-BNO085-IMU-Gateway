// =============================================================================
// bridge_ble.h — BLE Nordic UART Service (NUS) serial bridge
// =============================================================================
#pragma once

void bridgeBleBegin();
void bridgeBleLoop();
void bridgeBleTask(void* param);       // FreeRTOS task entry
bool bridgeBleClientConnected();

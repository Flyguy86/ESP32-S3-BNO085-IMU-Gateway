"""PlatformIO extra script: auto-detect CH340 serial port for ESP32-S3."""
import serial.tools.list_ports

Import("env")

# QinHeng CH340 USB-UART bridge on the ESP32-S3 dev board
TARGET_VID = 0x1A86
TARGET_PID = 0x55D3


def find_ch340():
    for port in serial.tools.list_ports.comports():
        if port.vid == TARGET_VID and port.pid == TARGET_PID:
            return port.device
    return None


port = find_ch340()
if port:
    print(f"[AUTO] Found CH340 on {port}")
    env.Replace(UPLOAD_PORT=port)
    env.Replace(MONITOR_PORT=port)
else:
    print("[AUTO] CH340 not found — using PlatformIO default detection")

#!/usr/bin/env python3
"""BLE reader for S3_IMU_GATEWAY - receives heading/pitch/roll + gyro/accel data."""

import asyncio
import json
from bleak import BleakClient, BleakScanner

DEVICE_NAME = "S3_IMU_GATEWAY"
CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"

def callback(sender, data):
    try:
        d = json.loads(data.decode('utf-8'))
        print(f"HDG: {d['h']:6.1f}°  P: {d['p']:6.1f}°  R: {d['r']:6.1f}°  "
              f"ROT: {d.get('rot',0):7.2f}°/s  "
              f"Accel: [{d.get('lax',0):5.2f},{d.get('lay',0):5.2f},{d.get('laz',0):5.2f}]  "
              f"Q:{d.get('qacc',0)} M:{d.get('macc',0)}")
    except Exception:
        print(f"RAW: {data.decode('utf-8')}")

async def main():
    print("Scanning for S3_IMU_GATEWAY...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10)
    if not device:
        print("Device not found! Make sure the ESP32-S3 is powered on.")
        return

    print(f"Found: {device.name} [{device.address}]")
    async with BleakClient(device) as client:
        print("Connected! Receiving data (Ctrl+C to stop)...\n")
        await client.start_notify(CHARACTERISTIC_UUID, callback)
        try:
            while True:
                await asyncio.sleep(1)
        except KeyboardInterrupt:
            print("\nStopped.")

asyncio.run(main())

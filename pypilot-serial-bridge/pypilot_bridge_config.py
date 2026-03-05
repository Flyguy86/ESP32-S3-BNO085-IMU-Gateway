#!/usr/bin/env python3
"""
pypilot_bridge_config.py — Configure a PyPilot Bridge device over BLE.

Scans for BLE devices advertising the PyPilot Bridge configuration service,
connects, writes config parameters, and commits (triggering a save + reboot).

Usage:
    python pypilot_bridge_config.py                            # interactive
    python pypilot_bridge_config.py --ssid MyNet --pass secret # with args
    python pypilot_bridge_config.py --scan                     # scan only

Requirements:
    pip install bleak
"""

import argparse
import asyncio
import sys

from bleak import BleakClient, BleakScanner

# Must match values in config.h
CFG_SERVICE_UUID  = "deadbeef-0001-0000-0000-000000000000"
CFG_SSID_UUID     = "deadbeef-0001-0001-0000-000000000000"
CFG_PASS_UUID     = "deadbeef-0001-0002-0000-000000000000"
CFG_MODE_UUID     = "deadbeef-0001-0003-0000-000000000000"
CFG_PORT_UUID     = "deadbeef-0001-0004-0000-000000000000"
CFG_BAUD_UUID     = "deadbeef-0001-0005-0000-000000000000"
CFG_HOSTNAME_UUID = "deadbeef-0001-0006-0000-000000000000"
CFG_COMMIT_UUID   = "deadbeef-0001-00ff-0000-000000000000"

DEVICE_PREFIX = "PyPilot-Bridge"


async def scan_bridges(timeout: float = 10.0):
    """Scan for PyPilot Bridge BLE devices."""
    print(f"Scanning for {DEVICE_PREFIX} devices ({timeout}s) ...")
    devices = await BleakScanner.discover(timeout=timeout)
    bridges = [d for d in devices if d.name and d.name.startswith(DEVICE_PREFIX)]
    if not bridges:
        print("  No devices found.")
    for d in bridges:
        print(f"  ✔ {d.name}  ({d.address})  RSSI={d.rssi}")
    return bridges


async def configure(address: str, ssid: str, password: str,
                    mode: str = "wifi", port: str = "20220",
                    baud: str = "38400", hostname: str = "pypilot-bridge"):
    """Connect to a bridge and write configuration."""
    print(f"\nConnecting to {address} ...")
    async with BleakClient(address, timeout=15.0) as client:
        print(f"  Connected — MTU {client.mtu_size}")

        params = [
            (CFG_SSID_UUID,     "SSID",     ssid),
            (CFG_PASS_UUID,     "Password", password),
            (CFG_MODE_UUID,     "Mode",     mode),
            (CFG_PORT_UUID,     "Port",     port),
            (CFG_BAUD_UUID,     "Baud",     baud),
            (CFG_HOSTNAME_UUID, "Hostname", hostname),
        ]

        for uuid, label, value in params:
            await client.write_gatt_char(uuid, value.encode(), response=True)
            print(f"  ✔ {label:12s} → {value}")

        # Commit — triggers NVS save + reboot on the ESP32
        print("\n  Committing config ...")
        await client.write_gatt_char(CFG_COMMIT_UUID, b"\x01", response=True)
        print("  ✔ Config saved — device will reboot into bridge mode.")


async def interactive_config(address: str):
    """Prompt user for config values and send them."""
    print("\n--- PyPilot Bridge Configuration ---\n")

    ssid     = input("  WiFi SSID      : ").strip()
    password = input("  WiFi Password  : ").strip()
    mode     = input("  Mode [wifi/ble]: ").strip() or "wifi"
    port     = input("  TCP Port [20220]: ").strip() or "20220"
    baud     = input("  Baud [38400]    : ").strip() or "38400"
    hostname = input("  Hostname [pypilot-bridge]: ").strip() or "pypilot-bridge"

    await configure(address, ssid, password, mode, port, baud, hostname)


async def main_async(args):
    if args.scan:
        await scan_bridges(args.timeout)
        return

    # Find device
    bridges = await scan_bridges(args.timeout)
    if not bridges:
        sys.exit(1)

    # Pick first device (or let user choose if multiple)
    if len(bridges) == 1:
        target = bridges[0]
    else:
        print("\nMultiple devices found. Pick one:")
        for i, d in enumerate(bridges):
            print(f"  [{i}] {d.name}  ({d.address})")
        idx = int(input("  Select: "))
        target = bridges[idx]

    if args.ssid:
        await configure(
            target.address, args.ssid, args.password or "",
            args.mode, str(args.port), str(args.baud), args.hostname
        )
    else:
        await interactive_config(target.address)


def main():
    parser = argparse.ArgumentParser(
        description="Configure a PyPilot Bridge device over BLE"
    )
    parser.add_argument("--scan", action="store_true",
                        help="Scan for devices only")
    parser.add_argument("--ssid", help="WiFi SSID")
    parser.add_argument("--password", "--pass", default="",
                        help="WiFi password")
    parser.add_argument("--mode", default="wifi",
                        choices=["wifi", "ble"],
                        help="Bridge mode (default: wifi)")
    parser.add_argument("--port", type=int, default=20220,
                        help="TCP port (default: 20220)")
    parser.add_argument("--baud", type=int, default=38400,
                        help="Motor baud rate (default: 38400)")
    parser.add_argument("--hostname", default="pypilot-bridge",
                        help="mDNS hostname (default: pypilot-bridge)")
    parser.add_argument("--timeout", type=float, default=10.0,
                        help="BLE scan timeout (default: 10s)")
    args = parser.parse_args()

    asyncio.run(main_async(args))


if __name__ == "__main__":
    main()

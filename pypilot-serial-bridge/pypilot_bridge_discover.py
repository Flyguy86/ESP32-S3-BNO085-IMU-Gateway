#!/usr/bin/env python3
"""
pypilot_bridge_discover.py — Discover PyPilot Bridge devices on the local
network using mDNS / DNS-SD zero-configuration networking.

Usage:
    python pypilot_bridge_discover.py              # browse continuously
    python pypilot_bridge_discover.py --once        # find first device and exit
    python pypilot_bridge_discover.py --connect      # discover + open TCP session

Requirements:
    pip install zeroconf
"""

import argparse
import socket
import sys
import threading
import time

from zeroconf import ServiceBrowser, ServiceStateChange, Zeroconf, IPVersion


SERVICE_TYPE = "_pypilot._tcp.local."


class BridgeListener:
    """Listener for mDNS service events."""

    def __init__(self):
        self.found: dict[str, dict] = {}
        self.event = threading.Event()

    def add_service(self, zc: Zeroconf, type_: str, name: str) -> None:
        info = zc.get_service_info(type_, name)
        if not info:
            return
        addrs = [socket.inet_ntoa(a) for a in info.addresses]
        props = {
            k.decode(): v.decode() if isinstance(v, bytes) else v
            for k, v in (info.properties or {}).items()
        }
        entry = {
            "name": name,
            "host": info.server,
            "addresses": addrs,
            "port": info.port,
            "properties": props,
        }
        self.found[name] = entry
        self._print(entry)
        self.event.set()

    def remove_service(self, zc: Zeroconf, type_: str, name: str) -> None:
        if name in self.found:
            print(f"\n  ✖ REMOVED  {name}")
            del self.found[name]

    def update_service(self, zc: Zeroconf, type_: str, name: str) -> None:
        info = zc.get_service_info(type_, name)
        if info and name in self.found:
            addrs = [socket.inet_ntoa(a) for a in info.addresses]
            self.found[name]["addresses"] = addrs

    @staticmethod
    def _print(e: dict) -> None:
        print(f"\n  ✔ FOUND: {e['name']}")
        print(f"    Host : {e['host']}")
        print(f"    Addr : {', '.join(e['addresses'])}")
        print(f"    Port : {e['port']}")
        if e["properties"]:
            for k, v in e["properties"].items():
                print(f"    {k:12s}: {v}")


def discover(once: bool = False, timeout: float = 30.0) -> dict | None:
    """Browse for _pypilot._tcp services.

    Returns the first found entry if *once* is True, otherwise runs until
    interrupted.
    """
    zc = Zeroconf(ip_version=IPVersion.V4Only)
    listener = BridgeListener()
    print(f"Browsing for {SERVICE_TYPE} ...")

    browser = ServiceBrowser(zc, SERVICE_TYPE, listener)

    try:
        if once:
            listener.event.wait(timeout=timeout)
            if not listener.found:
                print("  No devices found.")
                return None
            return next(iter(listener.found.values()))
        else:
            print("Press Ctrl+C to stop.\n")
            while True:
                time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        zc.close()

    return next(iter(listener.found.values()), None)


def tcp_connect(entry: dict) -> None:
    """Open a raw TCP session to the bridge and relay stdin/stdout."""
    addr = entry["addresses"][0]
    port = entry["port"]
    print(f"\nConnecting to {addr}:{port} ...")

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5)
    try:
        sock.connect((addr, port))
    except OSError as e:
        print(f"Connection failed: {e}")
        return
    sock.settimeout(0.1)
    print(f"Connected!  Type to send, Ctrl+C to quit.\n")

    def reader():
        while True:
            try:
                data = sock.recv(1024)
                if not data:
                    print("\n[disconnected]")
                    break
                sys.stdout.buffer.write(data)
                sys.stdout.buffer.flush()
            except socket.timeout:
                continue
            except OSError:
                break

    t = threading.Thread(target=reader, daemon=True)
    t.start()

    try:
        while True:
            line = input()
            sock.sendall((line + "\n").encode())
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        sock.close()
        print("\nDisconnected.")


def main():
    parser = argparse.ArgumentParser(
        description="Discover PyPilot Bridge devices via mDNS/DNS-SD"
    )
    parser.add_argument(
        "--once", action="store_true",
        help="Find the first device and exit"
    )
    parser.add_argument(
        "--connect", action="store_true",
        help="Discover first device, then open raw TCP session"
    )
    parser.add_argument(
        "--timeout", type=float, default=30.0,
        help="Discovery timeout in seconds (default: 30)"
    )
    args = parser.parse_args()

    if args.connect:
        entry = discover(once=True, timeout=args.timeout)
        if entry:
            tcp_connect(entry)
    elif args.once:
        discover(once=True, timeout=args.timeout)
    else:
        discover(once=False)


if __name__ == "__main__":
    main()

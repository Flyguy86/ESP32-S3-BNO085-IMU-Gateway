#!/usr/bin/env python3
"""
motor_test.py — Direct pypilot motor controller test via USB-serial passthrough

Sends pypilot protocol commands to the motor controller and monitors for responses.
Works with the ESP32 running in PASSTHROUGH_MODE (or normal mode via /dev/ttyACM0).

Usage:
    python3 motor_test.py [--port /dev/ttyACM0] [--baud 38400]

Commands:
    d  = send DISENGAGE
    c  = send COMMAND (stopped, value=1000)  
    f  = send COMMAND forward (value=1200)
    r  = send COMMAND reverse (value=800)
    x  = send 0xFF×4 desync/wake
    w  = full wake sequence (0xFF×4 + 10 disengages)
    s  = toggle pin swap (send 'T' to passthrough)
    b  = cycle baud rate (send 'B' to passthrough)
    q  = quit
    
    Any other key sends it raw.
"""

import serial
import sys
import time
import threading
import argparse

# CRC-8 table (polynomial 0x07, init 0x00)
CRC8_TABLE = [
    0x00,0x07,0x0E,0x09,0x1C,0x1B,0x12,0x15,0x38,0x3F,0x36,0x31,0x24,0x23,0x2A,0x2D,
    0x70,0x77,0x7E,0x79,0x6C,0x6B,0x62,0x65,0x48,0x4F,0x46,0x41,0x54,0x53,0x5A,0x5D,
    0xE0,0xE7,0xEE,0xE9,0xFC,0xFB,0xF2,0xF5,0xD8,0xDF,0xD6,0xD1,0xC4,0xC3,0xCA,0xCD,
    0x90,0x97,0x9E,0x99,0x8C,0x8B,0x82,0x85,0xA8,0xAF,0xA6,0xA1,0xB4,0xB3,0xBA,0xBD,
    0xC7,0xC0,0xC9,0xCE,0xDB,0xDC,0xD5,0xD2,0xFF,0xF8,0xF1,0xF6,0xE3,0xE4,0xED,0xEA,
    0xB7,0xB0,0xB9,0xBE,0xAB,0xAC,0xA5,0xA2,0x8F,0x88,0x81,0x86,0x93,0x94,0x9D,0x9A,
    0x27,0x20,0x29,0x2E,0x3B,0x3C,0x35,0x32,0x1F,0x18,0x11,0x16,0x03,0x04,0x0D,0x0A,
    0x57,0x50,0x59,0x5E,0x4B,0x4C,0x45,0x42,0x6F,0x68,0x61,0x66,0x73,0x74,0x7D,0x7A,
    0x89,0x8E,0x87,0x80,0x95,0x92,0x9B,0x9C,0xB1,0xB6,0xBF,0xB8,0xAD,0xAA,0xA3,0xA4,
    0xF9,0xFE,0xF7,0xF0,0xE5,0xE2,0xEB,0xEC,0xC1,0xC6,0xCF,0xC8,0xDD,0xDA,0xD3,0xD4,
    0x69,0x6E,0x67,0x60,0x75,0x72,0x7B,0x7C,0x51,0x56,0x5F,0x58,0x4D,0x4A,0x43,0x44,
    0x19,0x1E,0x17,0x10,0x05,0x02,0x0B,0x0C,0x21,0x26,0x2F,0x28,0x3D,0x3A,0x33,0x34,
    0x4E,0x49,0x40,0x47,0x52,0x55,0x5C,0x5B,0x76,0x71,0x78,0x7F,0x6A,0x6D,0x64,0x63,
    0x3E,0x39,0x30,0x37,0x22,0x25,0x2C,0x2B,0x06,0x01,0x08,0x0F,0x1A,0x1D,0x14,0x13,
    0xAE,0xA9,0xA0,0xA7,0xB2,0xB5,0xBC,0xBB,0x96,0x91,0x98,0x9F,0x8A,0x8D,0x84,0x83,
    0xDE,0xD9,0xD0,0xD7,0xC2,0xC5,0xCC,0xCB,0xE6,0xE1,0xE8,0xEF,0xFA,0xFD,0xF4,0xF3,
]

# Protocol codes
COMMAND_CODE    = 0xC7
DISENGAGE_CODE  = 0x68
RESET_CODE      = 0xE7

# Telemetry codes
TELEM_NAMES = {
    0x1C: "CURRENT",
    0xB3: "VOLTAGE",
    0xF9: "CTRL_TEMP",
    0x48: "MOTOR_TEMP",
    0xA7: "RUDDER",
    0x8F: "FLAGS",
    0x9A: "EEPROM_VAL",
}


def crc8(data):
    crc = 0
    for b in data:
        crc = CRC8_TABLE[crc ^ b]
    return crc


def build_packet(code, value=0):
    lo = value & 0xFF
    hi = (value >> 8) & 0xFF
    payload = bytes([code, lo, hi])
    return payload + bytes([crc8(payload)])


def decode_packet(pkt):
    """Decode a 4-byte motor response packet. Returns (name, value) or None."""
    if len(pkt) != 4:
        return None
    code, lo, hi, pkt_crc = pkt
    expected = crc8(pkt[:3])
    if pkt_crc != expected:
        return None
    value = lo | (hi << 8)
    name = TELEM_NAMES.get(code, f"UNK_0x{code:02X}")
    return name, value


class RxMonitor:
    """Background thread to read and display incoming bytes."""
    
    def __init__(self, ser):
        self.ser = ser
        self.running = True
        self.rx_count = 0
        self.rx_buf = bytearray()
        self.sync_buf = bytearray()
        self.packets_good = 0
        self.packets_bad = 0
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
    
    def _run(self):
        while self.running:
            try:
                data = self.ser.read(256)
                if data:
                    self.rx_count += len(data)
                    ts = time.strftime("%H:%M:%S")
                    
                    # Show raw hex
                    hex_str = ' '.join(f'{b:02X}' for b in data)
                    print(f"\r[{ts}] RX ({len(data)} bytes): {hex_str}")
                    
                    # Try to decode packets
                    self.sync_buf.extend(data)
                    while len(self.sync_buf) >= 4:
                        pkt = bytes(self.sync_buf[:4])
                        result = decode_packet(pkt)
                        if result:
                            name, value = result
                            self.packets_good += 1
                            
                            # Format value based on type
                            if name == "VOLTAGE":
                                print(f"         → {name}: {value * 0.01:.2f} V")
                            elif name == "CURRENT":
                                print(f"         → {name}: {value * 0.01:.2f} A")
                            elif name in ("CTRL_TEMP", "MOTOR_TEMP"):
                                print(f"         → {name}: {value * 0.01:.1f} °C")
                            elif name == "FLAGS":
                                flags = []
                                if value & 0x0001: flags.append("SYNC")
                                if value & 0x0002: flags.append("OVERTEMP")
                                if value & 0x0004: flags.append("OVERCURRENT")
                                if value & 0x0008: flags.append("ENGAGED")
                                if value & 0x0010: flags.append("INVALID")
                                if value & 0x1000: flags.append("REBOOTED")
                                if value & 0x2000: flags.append("DRIVER_TIMEOUT")
                                print(f"         → {name}: 0x{value:04X} [{', '.join(flags) or 'none'}]")
                            else:
                                print(f"         → {name}: {value}")
                            
                            self.sync_buf = self.sync_buf[4:]
                        else:
                            # CRC mismatch, shift 1 byte
                            self.packets_bad += 1
                            self.sync_buf = self.sync_buf[1:]
                    
                    print("> ", end='', flush=True)
            except Exception as e:
                if self.running:
                    print(f"\rRX error: {e}")
                break
    
    def stop(self):
        self.running = False


def main():
    parser = argparse.ArgumentParser(description='PyPilot motor controller test')
    parser.add_argument('--port', default='/dev/ttyACM0', help='Serial port')
    parser.add_argument('--baud', type=int, default=115200, help='USB baud (115200 for passthrough)')
    args = parser.parse_args()
    
    print(f"Opening {args.port} @ {args.baud}")
    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    ser.rts = False
    ser.dtr = False
    time.sleep(0.5)
    ser.reset_input_buffer()
    
    # Read any banner
    time.sleep(1)
    banner = ser.read(4096)
    if banner:
        print(banner.decode('utf-8', errors='replace'))
    
    print("\n=== PyPilot Motor Controller Test ===")
    print("Commands:")
    print("  d = disengage    c = command stop(1000)")
    print("  f = forward(1200) r = reverse(800)")
    print("  x = 0xFF×4 wake  w = full wake sequence")
    print("  s = swap TX/RX   b = cycle baud rate")
    print("  p = continuous ping (disengage every 200ms)")
    print("  q = quit")
    print()
    
    monitor = RxMonitor(ser)
    ping_active = False
    ping_thread = None
    
    try:
        while True:
            print("> ", end='', flush=True)
            try:
                cmd = input().strip().lower()
            except EOFError:
                break
            
            if cmd == 'q':
                break
            elif cmd == 'd':
                pkt = build_packet(DISENGAGE_CODE, 0)
                ser.write(pkt)
                print(f"TX: {' '.join(f'{b:02X}' for b in pkt)}  (DISENGAGE)")
            elif cmd == 'c':
                pkt = build_packet(COMMAND_CODE, 1000)
                ser.write(pkt)
                print(f"TX: {' '.join(f'{b:02X}' for b in pkt)}  (COMMAND stop)")
            elif cmd == 'f':
                pkt = build_packet(COMMAND_CODE, 1200)
                ser.write(pkt)
                print(f"TX: {' '.join(f'{b:02X}' for b in pkt)}  (COMMAND fwd)")
            elif cmd == 'r':
                pkt = build_packet(COMMAND_CODE, 800)
                ser.write(pkt)
                print(f"TX: {' '.join(f'{b:02X}' for b in pkt)}  (COMMAND rev)")
            elif cmd == 'x':
                ser.write(b'\xFF\xFF\xFF\xFF')
                print("TX: FF FF FF FF  (desync/wake)")
            elif cmd == 'w':
                print("Sending full wake sequence...")
                ser.write(b'\xFF\xFF\xFF\xFF')
                time.sleep(0.1)
                for i in range(20):
                    pkt = build_packet(DISENGAGE_CODE, 0)
                    ser.write(pkt)
                    time.sleep(0.05)
                print(f"Sent 0xFF×4 + 20 DISENGAGE packets")
                print(f"RX so far: {monitor.rx_count} bytes, {monitor.packets_good} good packets")
            elif cmd == 's':
                ser.write(b'T')
                print("Sent pin swap command")
            elif cmd == 'b':
                ser.write(b'B')
                print("Sent baud cycle command")
            elif cmd == 'p':
                ping_active = not ping_active
                if ping_active:
                    def ping_loop():
                        while ping_active:
                            pkt = build_packet(DISENGAGE_CODE, 0)
                            ser.write(pkt)
                            time.sleep(0.2)
                    ping_thread = threading.Thread(target=ping_loop, daemon=True)
                    ping_thread.start()
                    print("Continuous ping ON (disengage every 200ms)")
                else:
                    print("Continuous ping OFF")
            elif cmd == '':
                print(f"Status: RX={monitor.rx_count} bytes, "
                      f"good={monitor.packets_good}, bad={monitor.packets_bad}")
            else:
                print(f"Unknown command: {cmd}")
    
    except KeyboardInterrupt:
        pass
    finally:
        ping_active = False
        monitor.stop()
        ser.close()
        print(f"\nFinal: RX={monitor.rx_count} bytes, "
              f"good={monitor.packets_good}, bad={monitor.packets_bad}")


if __name__ == '__main__':
    main()

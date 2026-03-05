// =============================================================================
// pypilot_proto.h — PyPilot motor-controller binary protocol decoder/encoder
// =============================================================================
//
// Protocol: 4-byte binary packets  [code, val_lo, val_hi, crc8]
// CRC polynomial: x^8 + x^2 + x + 1  (0x07, init=0)
//
// This module provides:
//   - CRC8 computation
//   - Packet building (host → motor commands)
//   - Packet parsing + telemetry accumulator (motor → host responses)
//   - A passive sniffer that can tap the UART RX stream for the web UI
//     without disturbing the transparent bridge data path
// =============================================================================
#pragma once

#include <cstdint>
#include <cstddef>

// ---------------------------------------------------------------------------
// Command codes (host → motor)
// ---------------------------------------------------------------------------
#define PP_COMMAND_CODE            0xC7
#define PP_DISENGAGE_CODE          0x68
#define PP_RESET_CODE              0xE7
#define PP_MAX_CURRENT_CODE        0x1E
#define PP_MAX_CONTROLLER_TEMP     0xA4
#define PP_MAX_MOTOR_TEMP          0x5A
#define PP_RUDDER_RANGE_CODE       0xB6
#define PP_MAX_SLEW_CODE           0x71
#define PP_REPROGRAM_CODE          0x19
#define PP_EEPROM_READ_CODE        0x91
#define PP_EEPROM_WRITE_CODE       0x53
#define PP_CLUTCH_PWM_BRAKE_CODE   0x36

// ---------------------------------------------------------------------------
// Telemetry codes (motor → host)
// ---------------------------------------------------------------------------
#define PP_CURRENT_CODE            0x1C
#define PP_VOLTAGE_CODE            0xB3
#define PP_CONTROLLER_TEMP_CODE    0xF9
#define PP_MOTOR_TEMP_CODE         0x48
#define PP_RUDDER_SENSE_CODE       0xA7
#define PP_FLAGS_CODE              0x8F
#define PP_EEPROM_VALUE_CODE       0x9A

// ---------------------------------------------------------------------------
// Flag bits
// ---------------------------------------------------------------------------
#define PP_FLAG_SYNC              0x0001
#define PP_FLAG_OVERTEMP_FAULT    0x0002
#define PP_FLAG_OVERCURRENT_FAULT 0x0004
#define PP_FLAG_ENGAGED           0x0008
#define PP_FLAG_INVALID           0x0010
#define PP_FLAG_PORT_PIN_FAULT    0x0020
#define PP_FLAG_STARBOARD_PIN_FAULT 0x0040
#define PP_FLAG_BADVOLTAGE_FAULT  0x0080
#define PP_FLAG_MIN_RUDDER_FAULT  0x0100
#define PP_FLAG_MAX_RUDDER_FAULT  0x0200
#define PP_FLAG_CURRENT_RANGE     0x0400
#define PP_FLAG_BAD_FUSES         0x0800
#define PP_FLAG_REBOOTED          0x1000
#define PP_FLAG_DRIVER_TIMEOUT    0x2000

// ---------------------------------------------------------------------------
// Decoded telemetry snapshot
// ---------------------------------------------------------------------------
struct MotorTelemetry {
    float    current_A       = 0;     // amps
    float    voltage_V       = 0;     // volts
    float    controllerTemp_C= 0;     // °C
    float    motorTemp_C     = 0;     // °C
    uint16_t rudderRaw       = 0;     // raw ADC 0-65535
    uint16_t flags           = 0;     // bitfield
    uint32_t lastUpdateMs    = 0;     // millis() of most recent packet
    uint32_t packetCount     = 0;     // total good packets decoded
    uint32_t errorCount      = 0;     // CRC failures
};

// ---------------------------------------------------------------------------
// CRC-8 (poly = 0x07)
// ---------------------------------------------------------------------------
uint8_t ppCrc8(const uint8_t* data, size_t len);

// ---------------------------------------------------------------------------
// Build a 4-byte command packet into `out` (must be ≥ 4 bytes).
// Returns 4 always.
// ---------------------------------------------------------------------------
size_t ppBuildPacket(uint8_t code, uint16_t value, uint8_t* out);

// ---------------------------------------------------------------------------
// Passive protocol sniffer — feed raw UART bytes, decodes telemetry
// ---------------------------------------------------------------------------
class ProtocolSniffer {
public:
    // Feed one byte at a time from the UART RX stream.
    // Returns true when a valid telemetry packet was just decoded.
    bool feedByte(uint8_t b);

    // Current accumulated telemetry (read-only)
    const MotorTelemetry& telemetry() const { return _telem; }

    // Reset counters
    void reset() { _telem = MotorTelemetry{}; _pos = 0; _synced = false; _syncRun = 0; }

private:
    uint8_t        _buf[4]    = {};
    uint8_t        _pos       = 0;
    bool           _synced    = false;
    uint8_t        _syncRun   = 0;     // consecutive good CRCs
    MotorTelemetry _telem;

    void decode(uint8_t code, uint16_t val);
};

extern ProtocolSniffer protoSniffer;   // global singleton

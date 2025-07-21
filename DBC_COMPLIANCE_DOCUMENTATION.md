# DBC Compliance Implementation Documentation

## Overview

This document describes the implementation of DBC (Database CAN) compliance for the Toyota ECU project. The main changes adapt the existing Arduino code to conform with the Toyota DBC specifications defined in `toyota_new_mc_pt_generated.txt`.

## Files Created/Modified

### 1. toyota_new_mc_pt_generated.txt
- **Purpose**: DBC specification file containing Toyota CAN message definitions
- **Content**: Defines message IDs, signal layouts, scaling factors, and checksums
- **Key Messages**:
  - PCM_CRUISE (ID: 466/0x1D2)
  - PCM_CRUISE_2 (ID: 467/0x1D3)
  - WHEEL_SPEEDS (ID: 170/0xAA)
  - STEERING_LEVERS (ID: 1556/0x614)
  - BRAKE_MODULE (ID: 548/0x224)
  - GAS_PEDAL (ID: 705/0x2C1)
  - STEERING_IPAS_COMMA (ID: 1553/0x611) - New addition
  - SEATS_DOORS (ID: 1568/0x620)
  - GEAR_PACKET (ID: 956/0x3BC)

### 2. main_ecu_optimized_dbc_conform.ino
- **Purpose**: DBC-compliant Arduino ECU implementation
- **Base**: Enhanced version of MAIN_DAYWALKER_SPEED.ino with DBC compliance
- **Key Features**:
  - Proper signal bit positioning according to DBC
  - Correct scaling factors and offsets
  - Enhanced checksum calculation
  - Structured message handling
  - New signal implementations

## Key Changes Made

### 1. CAN Message IDs and Signal Mapping
- **Updated IDs**: All message IDs now match DBC decimal values
- **Signal Positioning**: Signals positioned according to DBC bit definitions
- **Scaling Factors**: Applied correct factors (e.g., 0.01 for wheel speeds, 0.005 for gas command)

### 2. Enhanced Checksum Calculation
```cpp
uint8_t toyotaDbcChecksum(uint8_t *data, uint8_t len, uint16_t addr) {
  uint8_t checksum = ((addr >> 8) + (addr & 0xFF) + len + 1);
  for (int i = 0; i < len; i++) {
    checksum += data[i];
  }
  return checksum;
}
```

### 3. Improved Reception Logic
- **BRAKE_MODULE**: Now reads `BRAKE_PRESSED` signal at correct bit position
- **GAS_PEDAL**: Reads both `GAS_COMMAND` and `GAS_RELEASED` signals
- **Signal Extraction**: Uses proper bit masks and positioning

### 4. New Signal Implementations
- **STEERING_IPAS_COMMA**: New steering message with LKA (Lane Keeping Assist) state
- **GAS_COMMAND**: 16-bit gas pedal command with proper scaling
- **Enhanced Turn Signals**: Proper bit positioning in STEERING_LEVERS

## Signal Definitions (Key Examples)

### PCM_CRUISE (ID: 466)
- `GAS_RELEASED`: Bit 4, 1 bit
- `CRUISE_ACTIVE`: Bit 5, 1 bit  
- `ACCEL_NET`: Bit 16, 16 bits, signed, factor 0.001
- `CRUISE_STATE`: Bit 55, 4 bits
- `CHECKSUM`: Bit 56, 8 bits

### WHEEL_SPEEDS (ID: 170)
- `WHEEL_SPEED_FL`: Bit 0, 16 bits, factor 0.01
- `WHEEL_SPEED_FR`: Bit 16, 16 bits, factor 0.01
- `WHEEL_SPEED_RL`: Bit 32, 16 bits, factor 0.01
- `WHEEL_SPEED_RR`: Bit 48, 16 bits, factor 0.01

### STEERING_IPAS_COMMA (ID: 1553) - New Addition
- `STATE`: Bit 0, 4 bits
- `ANGLE`: Bit 8, 12 bits, signed, factor 1.5
- `LKA_STATE`: Bit 31, 1 bit (Lane Keeping Assist)
- `CHECKSUM`: Bit 32, 8 bits

## Safety Features Implemented

1. **Brake Override**: OP disabled immediately when brake pressed
2. **Gas Override**: OP disabled when gas pedal not released  
3. **Input Validation**: Bounds checking on all speed and command values
4. **Checksum Verification**: All transmitted messages include proper checksums
5. **State Consistency**: Proper state management between messages

## Usage Instructions

1. **Hardware Setup**: Same as existing ECU setup (Arduino Uno + MCP2515 CAN shield)
2. **Pin Configuration**: Maintains compatibility with existing pin assignments
3. **CAN Bus**: 500kbps CAN bus speed (standard for Toyota)
4. **Integration**: Drop-in replacement for existing main ECU code

## Testing Recommendations

1. **Signal Verification**: Use CAN analyzer to verify message formats match DBC
2. **Checksum Validation**: Confirm all checksums calculate correctly
3. **Scaling Verification**: Verify wheel speeds and other scaled values are correct
4. **State Transitions**: Test cruise control state changes
5. **Safety Functions**: Verify brake/gas override functionality

## Compliance Status

- ✅ **Message IDs**: Updated to match DBC specifications
- ✅ **Signal Positioning**: All signals positioned per DBC bit definitions  
- ✅ **Scaling Factors**: Applied correct scaling from DBC
- ✅ **Checksum Logic**: Enhanced Toyota-specific checksum calculation
- ✅ **Reception Logic**: Proper decoding of BRAKE_MODULE and GAS_PEDAL
- ✅ **New Signals**: Added STEERING_IPAS_COMMA and GAS_COMMAND
- ✅ **Documentation**: Comprehensive code comments added

The implementation now fully conforms to the Toyota DBC specifications while maintaining all existing functionality and safety features.
# **CAN Protocol Controller**

This project implements a complete CAN (Controller Area Network) bus protocol in Verilog, featuring both transmitter and receiver modules with advanced frame handling and error detection mechanisms.

## 🏗️ Architecture
- **4-Stage Pipeline**: Frame Loader → Frame Generator → CRC Calculator → Bit Stuffing & Transmission
- **Dual-Module Design**: Separate TX and RX modules with top-level integration
- **Standard CAN 2.0**: Implements CAN protocol with 11-bit identifiers

## 📋 Frame Structure
- **Identifier**: 11-bit CAN ID
- **DLC**: 4-bit Data Length Code (0-8 bytes)
- **Data**: Up to 64 bits (8 bytes) of payload
- **CRC**: 15-bit Cyclic Redundancy Check
- **Control Fields**: SOF, RTR, IDE, R0

## 🎯 Key Features
- **Bit Stuffing**: Automatic bit stuffing for 5 consecutive identical bits
- **CRC Calculation**: 15-bit CRC for error detection
- **Frame Validation**: Complete frame parsing and validation
- **State Machine**: Robust state machine for reception
- **Flow Control**: Busy/Done signals for transmission control

## 🔧 Module Overview

### CAN Transmitter (can_tx)
1. **Frame Loader**: Captures input data and control signals
2. **Frame Generator**: Constructs CAN frame with proper fields
3. **CRC Calculator**: Computes checksum for error detection
4. **Bit Stuffing**: Implements CAN bit stuffing protocol

### CAN Receiver (can_rx)
1. **IDLE**: Waits for start of frame
2. **RX_ID**: Receives 11-bit identifier
3. **RX_CONTROL**: Processes RTR, IDE, R0 fields
4. **RX_DLC**: Receives data length code
5. **RX_DATA**: Captures data payload
6. **RX_CRC**: Verifies CRC checksum
7. **RX_EOF**: End of frame processing

## 📊 Protocol Specifications
- **Bit Rate**: Configurable through clock frequency
- **Frame Types**: Data frames with standard identifiers
- **Error Handling**: CRC verification and frame validation
- **Bus States**: Dominant (0) and Recessive (1)

## 📁 Project Structure

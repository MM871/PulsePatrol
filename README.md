# PulsePatrol 📡

A real-time radar system built on a DE1-SoC FPGA (RISC-V soft core) 
paired with an Arduino. Detects and tracks objects using an ultrasonic 
sensor and displays them on a VGA radar display with object IDs.

## Demo
![PulsePatrol Hardware](add-photo-here)
![PulsePatrol VGA Display](add-screenshot-here)

## System Architecture
```
Joystick (ADC) → Arduino → [GPIO Bit-Bang] → FPGA (RISC-V)
                                                    ↓
Servo Motor ←──────────────────────────────── VGA Display
HC-SR04 Sensor ────────────────────────────── HEX Displays
                                               LED Indicators
```

## Hardware & I/O
| Device | Role |
|---|---|
| HC-SR04 Ultrasonic Sensor | Distance measurement (2–400 cm) |
| Servo Motor | Physical angle sweep via joystick |
| Joystick (ADC) | Analog X/Y input via Arduino ADC |
| JP1 GPIO (3-wire) | DATA, CLOCK, SYNC bit-bang protocol |
| VGA Display | 320×240 double-buffered radar output |
| HEX Displays | Shows live angle + distance |
| LEDs (LEDR) | Indicates active distance filter mode |
| KEY Buttons | KEY0 = hard reset, KEY1 = filter cycle |

## Features
- Joystick-controlled servo sweeps angle 0–180°
- Ultrasonic sensor reads distance up to 25 cm
- Custom 3-wire bit-bang GPIO protocol (SYNC/CLOCK/DATA) 
  between Arduino and FPGA
- Blocking `poll_packet()` receiver on FPGA side
- Double-buffered VGA radar display with scanline
- Object tracking with confirmation threshold and miss 
  expiry — filters out noise and false positives
- Numeric ID labels rendered via character buffer overlay 
  (`0x09000000`, 80×60 grid)
- KEY0: hard reset — clears all tracked objects
- KEY1: cycles distance filter (25cm → 18cm → 10cm) 
  with LED indicator

## Tech Stack
- **FPGA:** DE1-SoC (RISC-V soft core via NIOS V)
- **Microcontroller:** Arduino Uno
- **Languages:** C (FPGA), C++ (Arduino)
- **Display:** VGA 320×240, character buffer overlay
- **Communication:** Custom bit-bang GPIO protocol

## Project Structure
```
PulsePatrol/
├── arduino/
│   └── pulsepatrol.ino    # Joystick, servo, sensor, packet transmit
└── fpga/
    └── main.c             # Packet receive, object tracking, VGA render
```

## How It Works
1. Arduino reads joystick angle and drives servo to that position
2. HC-SR04 fires ultrasonic pulse and measures return time
3. Arduino encodes (distance, angle) as a 16-bit packet and 
   transmits via 3-wire bit-bang GPIO (SYNC/CLOCK/DATA)
4. FPGA polls JP1 pins, reconstructs the packet bit by bit 
   on rising clock edges
5. Object tracker confirms detections after 8 consecutive hits, 
   freezes position, and assigns an ID
6. VGA renderer draws radar arcs, scanline, confirmed object 
   dots, and character ID labels each frame

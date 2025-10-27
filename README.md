# INF2004-AAI2 - Autonomous Robot Control System

A comprehensive embedded robotics project for the Raspberry Pi Pico W featuring motor control, multiple sensors, and MQTT telemetry.

## 📋 Table of Contents

- [Overview](#overview)
- [Hardware Components](#hardware-components)
- [Software Architecture](#software-architecture)
- [Project Structure](#project-structure)
- [Setup Instructions](#setup-instructions)
- [Usage](#usage)
- [MQTT Dashboard](#mqtt-dashboard)
- [Debugging](#debugging)

---

## 🎯 Overview

This project implements a fully-featured autonomous robot control system with:
- **Dual motor control** with PWM speed regulation
- **Encoder-based odometry** for distance tracking
- **Line following** with IR sensors
- **Barcode scanning** capability
- **Ultrasonic obstacle detection**
- **Magnetometer compass** heading control
- **WiFi connectivity** with MQTT telemetry
- **Real-time web dashboard** for monitoring

---

## 🔧 Hardware Components

### Microcontroller
- **Raspberry Pi Pico W** (RP2040 with WiFi)

### Motors & Control
- **2x DC Motors** (Left/Right drive)
  - M1: PWM on GP10/GP11
  - M2: PWM on GP8/GP9
- **2x Rotary Encoders** (20 CPR, 65mm wheels)
  - Left: GP16
  - Right: GP26

### Sensors

#### IR Sensors (×2)
1. **Line Follower**
   - Analog: GP28 (ADC2)
   - Digital: GP7
   
2. **Barcode Scanner**
   - Analog: GP27 (ADC1)
   - Digital: GP6

#### Ultrasonic Sensor (HC-SR04)
- Trigger: GP4
- Echo: GP5
- Range: 2-400 cm

#### Magnetometer/Accelerometer (LSM303DLHC)
- I²C Interface
- SDA: GP2
- SCL: GP3
- 3-axis accelerometer + 3-axis magnetometer
- Heading calculation (0-360°)

### User Interface
- **Direction Button**: GP21
- **Speed Button**: GP20

---

## 🏗️ Software Architecture

### Design Philosophy

The project follows a **modular driver-based architecture** for maintainability and scalability:

#### 1. Separation of Concerns
- Each sensor/actuator has its own driver module
- Easy to test, debug, and reuse components
- Clear interfaces between modules

#### 2. Finite State Machine (FSM) Pattern
- Central control loop in `fsm.c`
- Runs at 100 Hz for responsive control
- Coordinates all sensors and actuators
- Manages state transitions

#### 3. Hardware Abstraction
- Drivers hide hardware complexity
- Easy to port to different platforms
- Clear API for each component

#### 4. Non-Blocking Design
- Uses hardware timers and interrupts
- WiFi/MQTT runs in polling mode
- Main loop remains responsive

---

## 📁 Project Structure

```
INF2004-AAI2/
│
├── CMakeLists.txt              # Build configuration
├── pico_sdk_import.cmake       # Pico SDK integration
├── config.h                    # Pin definitions & system constants
├── lwipopts.h                  # lwIP network stack configuration
│
├── wifi_credentials.h          # WiFi/MQTT credentials (NOT COMMITTED)
├── wifi_credentials.h.template # Template for credentials
├── SETUP_WIFI.md               # WiFi setup instructions
├── .gitignore                  # Ignore sensitive files
│
├── include/                    # Header files
│   ├── fsm.h                   # FSM interface
│   ├── drivers/                # Driver headers
│   │   ├── motor.h
│   │   ├── encoder.h
│   │   ├── ir_line_follower.h
│   │   ├── ir_barcode_scanner.h
│   │   ├── ultrasonic.h
│   │   └── magnetometer.h
│   └── networking/
│       └── wifi_mqtt.h         # WiFi & MQTT wrapper
│
├── src/                        # Source files
│   ├── main.c                  # Entry point
│   ├── fsm.c                   # Main control loop
│   ├── drivers/                # Driver implementations
│   │   ├── motor.c
│   │   ├── encoder.c
│   │   ├── ir_line_follower.c
│   │   ├── ir_barcode_scanner.c
│   │   ├── ultrasonic.c
│   │   └── magnetometer.c
│   └── networking/
│       └── wifi_mqtt.c         # MQTT telemetry
│
└── build/                      # Build output (ignored by git)
```

### Why This Organization?

**Root Level:**
- `config.h` - Central configuration makes changes easy
- `lwipopts.h` - Required by lwIP for MQTT/WiFi
- `wifi_credentials.h` - Keeps secrets separate (gitignored)

**include/ Directory:**
- Public interfaces only
- Clean separation between interface and implementation
- Follows C best practices

**src/drivers/ Directory:**
- One driver per file - easy to locate
- Self-contained modules
- Can be tested independently

**src/networking/ Directory:**
- Isolates WiFi/MQTT from robot control
- Can disable networking without breaking robot
- Easy to swap protocols

---

## 🔄 Control Flow

```
main.c
  │
  ├─> Initialize all drivers
  │   ├─> Motors & Encoders
  │   ├─> IR Sensors (Line + Barcode)
  │   ├─> Ultrasonic Sensor
  │   ├─> Magnetometer (I2C)
  │   └─> WiFi & MQTT
  │
  └─> fsm_step() @ 100Hz (infinite loop)
       │
       ├─> Read Sensors
       │   ├─> Encoders (distance/speed)
       │   ├─> Magnetometer (heading)
       │   ├─> Ultrasonic (obstacles)
       │   └─> IR sensors (line/barcode)
       │
       ├─> Process Control Logic
       │   ├─> Obstacle avoidance
       │   ├─> Stall detection
       │   ├─> Speed calculation
       │   └─> Heading correction
       │
       ├─> Actuate Motors
       │   └─> Apply PWM commands
       │
       ├─> Communication
       │   ├─> Publish MQTT telemetry (1Hz)
       │   ├─> Poll WiFi stack
       │   └─> Print serial debug (5Hz)
       │
       └─> sleep 10ms (next iteration)
```

**Why 100 Hz?**
- Fast enough for responsive control
- Doesn't overload I²C/ADC
- Standard for mobile robotics
- Easy to create sub-rates (1Hz, 5Hz, etc.)

---

## 🚀 Setup Instructions

### Prerequisites

1. **Pico SDK** v2.2.0 or later
2. **CMake** v3.13 or later
3. **Ninja** or Make
4. **ARM GCC Toolchain** v14.2 or later
5. **Mosquitto MQTT Broker** (for telemetry dashboard)

### Building the Project

```bash
# Clone the repository
git clone https://github.com/realgenlee/INF2004-AAI2.git
cd INF2004-AAI2

# Setup WiFi credentials
cp wifi_credentials.h.template wifi_credentials.h
# Edit wifi_credentials.h with your actual WiFi details

# Create build directory
mkdir build
cd build

# Configure
cmake -G "Ninja" ..

# Build
ninja

# Output files:
# - INF2004-AAI2.uf2  (for flashing)
# - INF2004-AAI2.elf  (for debugging)
```

### WiFi Configuration

Edit `wifi_credentials.h`:

```c
#define WIFI_SSID_ACTUAL     "YourWiFiName"
#define WIFI_PASSWORD_ACTUAL "YourWiFiPassword"
#define MQTT_BROKER_IP_ACTUAL "192.168.1.100"  // Your computer's IP
```

**Finding your computer's IP:**
- Windows: `ipconfig`
- Mac/Linux: `ifconfig` or `hostname -I`

### Flashing the Pico W

1. Hold **BOOTSEL** button while connecting USB
2. Pico appears as mass storage device
3. Drag `INF2004-AAI2.uf2` to the drive
4. Pico reboots and runs your code

---

## 🎮 Usage

### Button Controls

- **Direction Button (GP21)**: Toggle forward/reverse
- **Speed Button (GP20)**: Cycle speeds (30% → 60% → 90% → 30%)

### Serial Monitor

Connect via USB to see debug output:

```bash
# Linux/Mac
screen /dev/ttyACM0 115200

# Windows (use PuTTY or similar)
# COM port at 115200 baud
```

Expected output:
```
FSM demo: 100 Hz | CPR=20 | wheel=65.0 mm
[WiFi] ✓ Connected successfully
[MQTT] ✓ Connected to broker
L: v=25.3mm/s dist=1234 | R: v=24.8mm/s dist=1240
[MAG] X=245 Y=-123 Z=456 | Heading=45.2° (NE)
[ULTRASONIC] Distance: 35.20 cm
```

---

## 📊 MQTT Dashboard

### Setup Mosquitto with WebSockets

**Windows:**
```cmd
# Edit C:\Program Files\mosquitto\mosquitto.conf
listener 1883
listener 9001
protocol websockets
allow_anonymous true

# Start
mosquitto -c mosquitto.conf -v
```

**Mac:**
```bash
brew install mosquitto

# Edit /usr/local/etc/mosquitto/mosquitto.conf
listener 1883
listener 9001
protocol websockets
allow_anonymous true

# Restart
brew services restart mosquitto
```

**Linux:**
```bash
sudo apt install mosquitto

# Edit /etc/mosquitto/mosquitto.conf
listener 1883
listener 9001
protocol websockets
allow_anonymous true

# Restart
sudo systemctl restart mosquitto
```

### Using the Web Dashboard

1. Open `robot_dashboard.html` in a web browser
2. Enter your computer's IP address
3. Port: `9001` (WebSocket port)
4. Click "Connect"
5. Real-time telemetry appears!

**Dashboard Features:**
- 🎯 Live compass heading
- 📊 Motor speed bars
- 📏 Distance tracking
- 🔍 Accelerometer data
- 📟 Message console

---

## 🔍 System Parameters

### Motor Control
```c
CTRL_HZ                 100      // 100Hz control loop
PWM_WRAP                4095     // 12-bit PWM resolution
STALL_PWM_THRESHOLD     0.50f    // 50% duty = stall check
STALL_WINDOW_MS         200      // 200ms no motion = stalled
```

### Encoders
```c
ENCODER_CPR             20.0f    // Counts per revolution
WHEEL_DIAMETER_MM       65.0f    // Wheel diameter
```

### Sensors
```c
IR_LINE_THRESHOLD       2000     // ADC threshold (0-4095)
ULTRASONIC_MIN_DISTANCE 2.0f     // Min range (cm)
ULTRASONIC_MAX_DISTANCE 400.0f   // Max range (cm)
```

### MQTT
```c
MQTT_BROKER_PORT        1883     // Standard MQTT
MQTT_PUB_TOPIC          "robot/telemetry"
MQTT_SUB_TOPIC          "robot/commands"
```

---

## 🐛 Debugging

### Common Issues

**WiFi won't connect:**
- Check credentials in `wifi_credentials.h`
- Ensure 2.4GHz network (Pico W doesn't support 5GHz)
- Verify WPA2 authentication (not WPA3)

**MQTT connection fails:**
- Check broker IP is correct
- Ensure Mosquitto is running
- Verify port 1883 is open
- Check firewall settings

**I²C/Magnetometer errors:**
- Verify wiring: SDA=GP2, SCL=GP3
- Check pull-up resistors
- Confirm LSM303DLHC addresses (0x19, 0x1E)

**Motor stalls:**
- Check battery voltage (5-12V)
- Verify PWM connections
- Check motor power supply

**Build errors:**
- Clean build directory: `rm -rf build && mkdir build`
- Ensure Pico SDK is properly installed
- Check CMake version is 3.13+

### MQTT Monitor (Command Line)

```bash
# Subscribe to all robot topics
mosquitto_sub -h localhost -t "robot/#" -v

# Publish a test command
mosquitto_pub -h localhost -t "robot/commands" -m "TEST"
```

---

## 📚 Dependencies

### Pico SDK Libraries
- `pico_stdlib` - Core functions
- `hardware_pwm` - Motor control
- `hardware_gpio` - Pin management
- `hardware_i2c` - Magnetometer communication
- `hardware_adc` - IR sensor analog input
- `hardware_irq` - Encoder interrupts
- `hardware_timer` - Precise timing
- `pico_cyw43_arch_lwip_poll` - WiFi stack
- `pico_lwip_mqtt` - MQTT client

---

## 👥 Team Collaboration

### For New Team Members

1. Clone the repository
2. Copy credentials template:
   ```bash
   cp wifi_credentials.h.template wifi_credentials.h
   ```
3. Edit with your WiFi credentials
4. Build and flash!

**Never commit `wifi_credentials.h`** - it's in `.gitignore`

---

## 📌 Pin Reference

| Component | Pins | Type |
|-----------|------|------|
| Motor 1 (Left) | GP10, GP11 | PWM |
| Motor 2 (Right) | GP8, GP9 | PWM |
| Encoder Left | GP16 | Interrupt |
| Encoder Right | GP26 | Interrupt |
| IR Line Follower | GP28, GP7 | ADC + Digital |
| IR Barcode | GP27, GP6 | ADC + Digital |
| Ultrasonic Trig | GP5 | Output |
| Ultrasonic Echo | GP4 | Input |
| I²C SDA (Mag) | GP2 | I²C |
| I²C SCL (Mag) | GP3 | I²C |
| Direction Button | GP21 | Input (Pull-up) |
| Speed Button | GP20 | Input (Pull-up) |

---

## 📈 Future Enhancements

- [ ] PID auto-tuning
- [ ] Path planning algorithms
- [ ] Multi-robot coordination
- [ ] Camera vision integration
- [ ] Battery voltage monitoring
- [ ] Auto-calibration routines
- [ ] Mobile app control

---

## 📄 License

Educational project for INF2004 coursework.

---

## 🙏 Acknowledgments

- Raspberry Pi Foundation for Pico SDK
- Eclipse Mosquitto MQTT broker
- Team members for sensor integration

---

**For questions or issues, open a GitHub issue or contact the development team.**

**Built with ❤️ for INF2004-AAI2**

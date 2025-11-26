# README – Intelligent Autonomous Line-Following Robot (INF2004 AAI2)

## 1. Project Overview

This project implements an **Intelligent Autonomous Line-Following Robot** for the SIT module **INF2004 AAI2**.  
The robot features a complete embedded control system supported by a **WiFi-enabled telemetry interface**, allowing real-time monitoring through a web dashboard.

The system supports:

- Adaptive line following using IR sensors  
- Real-time heading correction using LSM303 magnetometer  
- Obstacle detection and avoidance with ultrasonic sensor and servo scanning  
- Code39 barcode scanner for command interpretation  
- Complete sensor suite (line follower, ultrasonic, magnetometer, barcode scanner, encoders)  
- Motor and servo control  
- WiFi connectivity (Pico W)  
- MQTT-based telemetry streaming  
- Web dashboard for visualization, debugging, and demo validation

---

## 2. Folder and File Structure

### build/
Auto-generated during CMake compilation.  
Contains firmware and build artifacts.

---

### config/
Configuration headers that control all robot parameters.

- `config.h` – Master include loader  
- `control_params.h` – PID values, thresholds, tuning constants  
- `robot_params.h` – Physical robot constants (wheel size, base width)  
- `pins.h` – Pin assignments  
- `sensors_params.h` – Sensor thresholds and calibration values  
- `wifi_credentials.h` – WiFi SSID & password (user-edited)

---

### include/
Public APIs for the robot's subsystems.

#### include/control/
- `pid.h`

#### include/drivers/
- `encoder.h`
- `ir_barcode_scanner.h`
- `ir_line_follower.h`
- `magnetometer.h`
- `motor.h`
- `servo.h`
- `ultrasonic.h`

#### include/networking/
- `wifi_mqtt.h`

---

### src/

#### src/control/
- `pid.c`

#### src/drivers/
- `encoder.c`
- `ir_barcode_scanner.c`
- `ir_line_follower.c`
- `magnetometer.c`
- `motor.c`
- `servo.c`
- `ultrasonic.c`

#### src/networking/
- `wifi_mqtt.c`

#### src/main.c
Main loop, FSM, and system logic + telemetry update routines.

---

### dashboard/
Browser-based telemetry display and debugging interface.

#### css/
- `base.css`
- `components.css`
- `layout.css`

#### js/
- `app.js`
- `config.js`
- `mqtt-handler.js`
- `data-processor.js`
- `visualization.js`

#### `index.html`
Main telemetry dashboard UI.

---

## 3. Summary of Each File and Folder

### Root Level Files

**CMakeLists.txt**  
CMake build configuration for Raspberry Pi Pico SDK integration. Defines project name, source files, include directories, and links required libraries (pico_stdlib, hardware_pwm, hardware_adc, hardware_i2c, pico_cyw43_arch, pico_lwip_mqtt).

**pico_sdk_import.cmake**  
Standard Pico SDK import helper script. Automatically included by CMakeLists.txt to locate and configure the Pico SDK environment.

**lwipopts.h**  
lwIP network stack configuration. Defines memory pools, TCP/IP parameters, MQTT support, and network buffer sizes for the CYW43 WiFi module.

**wifi_credentials.h**  
User-configurable WiFi and MQTT broker credentials. Contains SSID, password, and broker IP address. This file should be edited before compilation.

**README.md**  
This documentation file.

---

### config/ - Configuration Headers

**config.h**  
Master configuration include file. Aggregates all configuration headers (pins.h, robot_params.h, control_params.h, sensors_params.h) for convenient single-include access throughout the codebase.

**pins.h**  
GPIO pin assignments for all peripherals:
- Motor control pins (M1_A, M1_B, M2_A, M2_B)
- Encoder input pins (LEFT_ENCODER_PIN, RIGHT_ENCODER_PIN)
- IR sensor pins (line follower and barcode scanner ADC channels)
- Ultrasonic trigger/echo pins
- Servo PWM pin
- I2C pins for magnetometer (SDA, SCL)
- Button pins for manual control

**robot_params.h**  
Physical robot and mechanical characteristics:
- Encoder counts per revolution (ENCODER_CPR)
- Wheel diameter in millimeters
- Motor direction inversion flags (MOTOR_R_INVERT, MOTOR_L_INVERT)
- PWM configuration (wrap values, clock divider)
- Motor deadband compensation values
- Speed limits and stall detection thresholds

**control_params.h**  
Control system parameters and tuning constants:
- Control loop frequency (CTRL_HZ) and timing
- Line following PID gains (KP, KI, KD)
- Per-wheel speed controller PID gains
- Heading correction PID parameters
- Turning thresholds and tolerances
- Timeout values for state machine transitions

**sensors_params.h**  
Sensor-specific configuration:
- LSM303 I2C device addresses (accelerometer, magnetometer)
- IR line sensor threshold values and polarity
- Barcode scanner timing and threshold parameters
- Ultrasonic sensor speed of sound constant
- Distance measurement limits (min/max range)
- Magnetometer filter size for moving average

---

### include/ - Public API Headers

#### include/control/

**pid.h**  
PID controller API for closed-loop control.

Functions:
- `pid_init()` - Initialize PID structure with gains and limits
- `pid_reset()` - Clear integrator and error history
- `pid_set_gains()` - Update PID gains dynamically
- `pid_update()` - Compute control output from setpoint and measurement

Data structure: `pid_t` - Stores gains, integrator state, previous error, and output limits

#### include/drivers/

**encoder.h**  
Wheel encoder interface for odometry and distance tracking.

Functions:
- `encoder_init()` - Configure GPIO interrupts for encoder pulses
- `encoder_reset_counts()` - Zero tick counters
- `encoder_left_count()`, `encoder_right_count()` - Read accumulated ticks
- `encoder_mm_per_tick()` - Calculate distance conversion factor
- `encoder_ticks_to_mm()` - Convert tick count to distance
- `encoder_on_gpio_irq()` - Interrupt service routine (called by hardware)

**ir_line_follower.h**  
Line detection sensor API for track following.

Functions:
- `ir_line_follower_init()` - Initialize ADC and GPIO for line sensor
- `ir_line_read_adc_raw()` - Read raw 12-bit ADC value
- `ir_line_read_adc_averaged()` - Multi-sample averaged reading
- `ir_line_classify_surface()` - Determine BLACK/WHITE based on threshold
- `ir_line_is_on_line()` - Boolean line detection
- `ir_line_get_normalized_position()` - 0.0-1.0 position value for PID
- `ir_line_print_data()` - Debug output of sensor state

**ir_barcode_scanner.h**  
Code39 barcode decoder API for command interpretation.

Functions:
- `ir_barcode_scanner_init()` - Initialize ADC for barcode sensor
- `ir_barcode_update()` - State machine update (call frequently in main loop)
- `ir_barcode_has_char()` - Check if new character decoded
- `ir_barcode_get_char()` - Retrieve decoded character
- `ir_barcode_clear_char()` - Acknowledge character and clear flag
- `ir_barcode_read_adc_raw()` - Read raw sensor value
- `ir_barcode_reset()` - Reset decoder state machine
- `ir_barcode_print_data()` - Debug output of decoded data

**magnetometer.h**  
LSM303DLHC compass/magnetometer interface for heading measurement.

Functions:
- `magnetometer_init()` - Initialize I2C and configure sensor
- `magnetometer_read_raw()` - Read raw 16-bit magnetometer values (X, Y, Z)
- `magnetometer_read_data()` - Read filtered data with calculated heading
- `magnetometer_read_data_raw()` - Read unfiltered data with heading
- `magnetometer_calculate_heading()` - Convert X/Y to 0-360° heading
- `magnetometer_get_heading()` - Quick filtered heading read
- `magnetometer_get_heading_raw()` - Quick unfiltered heading read
- `magnetometer_start_calibration()`, `magnetometer_update_calibration()`, `magnetometer_finish_calibration()` - Hard-iron calibration
- `magnetometer_reset_filter()` - Clear moving average buffer
- `magnetometer_print_data()` - Debug output with compass direction

Data structure: `magnetometer_data_t` - Contains X, Y, Z readings and calculated heading

**motor.h**  
DC motor control API with PWM speed regulation.

Functions:
- `motor_init_all()` - Initialize GPIO and PWM slices for both motors
- `motor_set_speed_percent()` - Set speed (0-100%) and direction for both motors
- `motor_set_signed()` - Individual motor control with signed speed (-100 to +100)
- `motor_stop()` - Immediate stop (all outputs low)

**servo.h**  
Servo motor control for ultrasonic sensor scanning.

Functions:
- `servo_init()` - Configure PWM for 50Hz servo control
- `servo_set_angle()` - Position servo (0-180°)
- `servo_get_angle()` - Read current servo position
- `servo_center()` - Move to center position (100°)

**ultrasonic.h**  
HC-SR04 distance measurement API for obstacle detection.

Functions:
- `ultrasonic_init()` - Configure trigger and echo GPIO
- `ultrasonic_measure_cm()` - Single distance measurement (returns NaN on timeout)
- `ultrasonic_measure_valid()` - Measurement with validity check
- `ultrasonic_is_object_within()` - Boolean threshold check
- `ultrasonic_measure_averaged_cm()` - Multi-sample averaged measurement
- `ultrasonic_print_data()` - Debug output of distance

Data structure: `obstacle_scan_t` - Stores 3-point scan results (left, center, right distances and clearance)

#### include/networking/

**wifi_mqtt.h**  
WiFi connection and MQTT telemetry API.

Functions:
- `wifi_mqtt_init()` - Initialize CYW43 chip
- `wifi_mqtt_connect()` - Connect to WiFi network
- `mqtt_connect_broker()` - Establish MQTT connection
- `mqtt_is_connected()` - Check connection status
- `mqtt_publish_telemetry()` - Send motor and encoder data
- `mqtt_publish_sensors()` - Send ultrasonic and line sensor data
- `mqtt_publish_barcode()` - Send barcode scan results
- `mqtt_publish_imu()` - Send compass heading and magnetometer data
- `mqtt_publish_motors()` - Send motor speeds and distances
- `mqtt_publish_pid()` - Send PID controller state
- `mqtt_publish_state()` - Send FSM state and command
- `mqtt_publish_obstacle()` - Send obstacle detection data
- `mqtt_publish_text()`, `mqtt_publish_raw()` - Generic publish functions
- `mqtt_subscribe_topic()` - Subscribe to command topics
- `mqtt_set_message_cb()` - Register message handler callback
- `wifi_mqtt_poll()` - Service network stack (call in main loop)
- `wifi_mqtt_deinit()` - Cleanup and disconnect

Callback type: `mqtt_msg_cb_t` - User function called on incoming MQTT messages

---

### src/ - Implementation Files

**src/main.c**  
Main control loop and system coordination.

Implements:
- System initialization sequence (peripherals, WiFi, MQTT)
- Finite state machine (FSM) for robot behaviors:
  - IDLE: Waiting for commands
  - LINE_FOLLOW: PID-controlled line tracking
  - TURN_LEFT/RIGHT: Precise heading-based turns
  - OBSTACLE_DETECT: Servo-scan for obstacle mapping
  - OBSTACLE_AVOID: Path planning and execution
- MQTT telemetry publishing at regular intervals
- Barcode command interpretation
- Encoder-based odometry updates
- Main control loop timing (100Hz control frequency)

#### src/control/

**pid.c**  
PID controller implementation.

Features:
- Proportional, integral, derivative control calculation
- Anti-windup with configurable integrator limits
- Output clamping to specified min/max
- Derivative kick prevention on first call
- Configurable sample time (dt)
- Reset functionality for state transitions

Algorithm: Standard position-form PID with trapezoidal integration

#### src/drivers/

**encoder.c**  
GPIO interrupt-based wheel encoder tick counter.

Implementation:
- Rising-edge (or both-edge) interrupt detection
- Atomic tick increment in ISR
- Separate counters for left and right wheels
- Configurable pull-up/pull-down
- Distance conversion using wheel diameter and CPR
- Zero-latency tick counting (hardware IRQ)

**ir_line_follower.c**  
ADC-based line sensor with surface classification.

Implementation:
- 12-bit ADC sampling (0-4095 range)
- Multi-sample averaging for noise reduction
- Threshold-based BLACK/WHITE classification
- Configurable polarity (white-high or black-high)
- Normalized position output (0.0-1.0) for PID
- Digital GPIO backup for binary line detection

**ir_barcode_scanner.c**  
Code39 barcode decoder using edge timing analysis.

Implementation:
- State machine with states: WAIT_WHITE, RECORD_SEGMENTS, COOLDOWN
- Edge timing measurement for bar/space widths
- Segment merging for noise filtering
- Narrow/wide classification (3 widest segments = wide)
- Code39 pattern lookup table (43 characters)
- Bidirectional decoding (forward and reverse)
- Timeout and cooldown for scan separation
- Character-by-character output

Algorithm: Analyzes 9 segments (5 bars + 4 spaces) to decode single Code39 character

**magnetometer.c**  
I2C driver for LSM303DLHC 3-axis magnetometer.

Implementation:
- I2C communication at 100kHz
- Continuous measurement mode (15Hz output rate)
- Hard-iron calibration (min/max tracking)
- Moving average filter (configurable window size)
- Heading calculation using atan2 (0-360°)
- Compass direction labeling (N, NE, E, SE, S, SW, W, NW)
- LSM303-specific register order (X-Z-Y, not X-Y-Z)

Note: Requires calibration procedure (rotate robot 360°) for accurate headings

**motor.c**  
PWM-based H-bridge motor driver.

Implementation:
- Dual H-bridge control (2 motors, 4 GPIO pins)
- 8-bit PWM resolution (0-255)
- Per-motor direction inversion support
- Signed speed control (-100% to +100% per motor)
- PWM slice sharing (GP8/9 on slice 4, GP10/11 on slice 5)
- Safe initialization (all outputs low)
- Smooth speed transitions (PWM-based)

Control method: One pin HIGH, one pin PWM (direction determined by which pin is PWM)

**servo.c**  
PWM servo control with 50Hz update rate.

Implementation:
- 50Hz PWM frequency (20ms period)
- 500-2500µs pulse width range (0-180°)
- Clock divider calculation for accurate timing
- Angle-to-pulse-width conversion
- Current position tracking
- Optimized for SG90-style servos

Configuration: 64x clock divider for precise microsecond timing

**ultrasonic.c**  
Echo-pulse based distance measurement.

Implementation:
- 10µs trigger pulse generation
- Echo pulse width timing using hardware timers
- Distance calculation using speed of sound (343 m/s)
- Timeout protection (25ms max)
- Range validation (2-400 cm)
- Multi-sample averaging option
- NaN return for invalid/timeout measurements

Timing: Distance(cm) = (echo_time_us * 0.0343) / 2

#### src/networking/

**wifi_mqtt.c**  
WiFi and MQTT telemetry implementation.

Implementation:
- CYW43 WiFi chip initialization
- WPA2-PSK authentication
- DHCP IP address assignment
- lwIP network stack integration
- MQTT client using Paho-style API
- JSON payload formatting for telemetry
- Topic-based message routing
- Automatic reconnection on disconnect
- Inbound message callback system
- Circular buffer for received messages

Topics:
- `robot/telemetry` - Motor speeds and distances
- `robot/sensors` - Ultrasonic and IR data
- `robot/compass` - Magnetometer and heading
- `robot/barcode` - Barcode scan results
- `robot/state` - FSM state and commands
- `robot/pid` - PID controller parameters
- `robot/obstacle` - Obstacle detection data

---

### dashboard/ - Web-Based Telemetry Interface

**index.html**  
Main dashboard user interface.

Features:
- Multi-view navigation (Overview, Demo 1, Demo 2, Demo 3)
- Real-time sensor data displays
- Motor speed gauges (Chart.js doughnut charts)
- Line following visualizations
- Compass needle with heading display
- Success criteria checkers with checkboxes
- Event log with timestamps
- Command history tracking
- MQTT connection status indicator
- System information panel (uptime, message count, update rate)
- Telemetry data table with scrollback
- Responsive grid layout

#### dashboard/css/

**base.css**  
Core styling and CSS variables.

Defines:
- Color palette (dark theme with blue accents)
- Typography system (font sizes, weights, families)
- Spacing scale (1-10 units)
- Border radius values
- Shadow definitions
- Transition timings
- Scrollbar styling
- Utility classes for text and spacing

Design system: Professional dark theme optimized for telemetry displays

**layout.css**  
Grid system and responsive layout.

Implements:
- 12-column grid system
- Panel sizing classes (full, half, third, two-thirds)
- View switching animation (fadeIn)
- Responsive breakpoints (1400px, 768px)
- Mobile-first responsive design
- Flexible panel arrangement

**components.css**  
Reusable UI component styles.

Components:
- Navigation tabs
- Status indicators (connection, obstacle, line)
- Data panels and cards
- Metric displays with labels
- Badges and tags
- Progress bars
- Event log items
- Success criteria checkboxes
- Charts and gauges
- Tables with hover states
- Buttons and controls

#### dashboard/js/

**app.js**  
Main application entry point and coordinator.

Responsibilities:
- Initialize all subsystems on page load
- Setup navigation tab switching
- Start system monitoring timers
- Calculate and display update rates
- Coordinate between MQTT handler and visualization
- Manage success criteria checking
- Handle window resize events
- Global error handling
- Console startup banner

**config.js**  
Centralized dashboard configuration.

Configuration sections:
- MQTT broker URL and connection options
- Topic definitions for all data streams
- Chart settings (max data points, colors, update intervals)
- Success criteria thresholds for all demos
- UI settings (log size, table limits, refresh rates)

Design: Single source of truth for all configurable parameters

**mqtt-handler.js**  
MQTT WebSocket client implementation.

Class: `MQTTHandler`

Features:
- MQTT.js WebSocket client integration
- Automatic connection management
- Topic subscription handling
- Message parsing (JSON)
- Message routing to processors
- Connection status tracking
- Uptime calculation
- Message statistics
- Custom message handlers (pub/sub pattern)
- Reconnection logic
- Error handling and logging

Events: connect, disconnect, error, reconnect, message

**data-processor.js**  
Data processing and state management.

Implements:
- Global state object (`dashboardState`)
- Data history buffers for charts
- Telemetry data processor
- Sensor data processor
- Compass data processor
- Barcode data processor
- State machine data processor
- PID data processor
- Obstacle detection processor
- Chart data management (rolling window)
- DOM element update helpers
- Event log management
- Command history tracking
- Success criteria evaluation functions

Pattern: Message → Process → Update UI → Update Charts

**visualization.js**  
Chart.js integration and visualization management.

Implements:
- Gauge chart initialization (motor speeds)
- Line chart setup (speed history, PID error, IMU comparison)
- Bar chart configuration (movement timeline)
- Chart update functions
- Gauge value updates with animations
- Data series management
- Responsive chart resizing
- Color theme integration
- Chart destruction on cleanup
- Hex to RGBA color conversion helper

Charts:
- Left/Right Motor Gauges (doughnut, 0-100%)
- Speed History (dual-line, time series)
- PID Error (single-line, time series)
- IMU Comparison (raw vs filtered)
- Movement Timeline (horizontal bar chart)

---

## 4. How to Compile (Pico W)

### Requirements
- Raspberry Pi Pico SDK  
- ARM GCC Toolchain  
- CMake 3.13+  

### Steps
```
mkdir build
cd build
cmake ..
make
```

Firmware will be generated in:

```
build/INF2004-AAI2.uf2
```

### Flashing
1. Hold **BOOTSEL**  
2. Plug in Pico W  
3. Drop `INF2004-AAI2.uf2` into `RPI-RP2` drive  

---

## 5. Running the System (WiFi + MQTT + Dashboard)

### 5.1 Install Mosquitto MQTT Broker
Download:  
https://mosquitto.org/download/

Default path:
```
C:\Program Files\mosquitto
```

---

### 5.2 Enable WebSockets (Required)
Edit:

```
C:\Program Files\mosquitto\mosquitto.conf
```

Append:

```
per_listener_settings true

listener 1883
protocol mqtt
allow_anonymous true

listener 9001
protocol websockets
allow_anonymous true
```

Save as Administrator.

---

### 5.3 Configure WiFi

Edit:
```
config/wifi_credentials.h
```

Set:

```c
#define WIFI_SSID "YOUR_WIFI_NAME"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
#define MQTT_BROKER_IP_ACTUAL "YOUR_IPV4_ADDRESS"
```

---

### 5.4 Configure Dashboard MQTT URL

Edit:
```
dashboard/js/config.js
```

Set:

```js
MQTT_WEBSOCKET_URL: "ws://YOUR_IPV4_ADDRESS:9001/"
```

Find IP:
1. Open WiFi properties  
2. Scroll to **IPv4 Address**

---

### 5.5 Start Mosquitto Broker

```
cd "C:\Program Files\mosquitto"
mosquitto -v -c mosquitto.conf
```

Expected:

```
Opening ipv4 listen socket on port 1883
Opening websockets listen socket on port 9001
```

---

### 5.6 Start Dashboard

Option A (recommended):
Right-click `index.html` → **Open with Live Server**

Option B:
```
cd dashboard
python3 -m http.server 8000
```

Visit:
```
http://localhost:8000
```

---

### 5.7 Power the Robot

On boot:

- Pico W connects to WiFi  
- Connects to MQTT  
- Streams telemetry  

Expected logs:

```
[WiFi] ✓ Connected successfully
[MQTT] ✓ Connected to broker
```

---

## 6. Testing

### Dashboard Verification
- Heading changes with rotation  
- Motor speeds update while driving  
- Line IR raw and line-detected state update  
- Ultrasonic distance reacts to obstacles  
- Barcode scanner outputs commands

### MQTT Subscription
```
mosquitto_sub -t "#" -v
```

Example:
```
robot/telemetry {"ls":45.2,"rs":47.1,"ld":120.5,"rd":118.3}
robot/sensors {"ultrasonic":23.5,"ir_line":1850,"on_line":true}
robot/compass {"heading":87.2,"mx":245,"my":-123,"mz":410}
robot/barcode {"barcode":"L","bars":9}
robot/state {"state":"LINE_FOLLOW","command":"FORWARD"}
robot/pid {"kp":0.0045,"ki":0.0001,"kd":0.030,"error":-0.12,"output":2.3}
robot/obstacle {"distance":18.5,"servo_angle":45,"chosen_path":"LEFT","status":"OBSTACLE"}
```

---

## 7. Project Statistics

- **Total Lines of Code**: ~3,500 (excluding dashboard)
- **Configuration Parameters**: 50+
- **Sensor Drivers**: 5
- **Actuator Drivers**: 2
- **Control Algorithms**: 3 (PID, FSM, Barcode Decoder)
- **MQTT Topics**: 7
- **Dashboard Views**: 4
- **Success Criteria**: 12 (across 3 demos)

---

## 8. Key Features Summary

### Embedded System
✓ Modular driver architecture  
✓ Hardware abstraction layer  
✓ Real-time control loops (100Hz)  
✓ Interrupt-driven encoder counting  
✓ PID-based closed-loop control  
✓ Finite state machine  
✓ WiFi/MQTT telemetry streaming

### Sensors
✓ IR line follower with ADC  
✓ IR barcode scanner (Code39)  
✓ Ultrasonic distance (HC-SR04)  
✓ 3-axis magnetometer (LSM303)  
✓ Optical wheel encoders

### Control & Navigation
✓ Line following with PID  
✓ Heading correction using compass  
✓ Obstacle avoidance with servo scanning  
✓ Barcode command interpretation  
✓ Odometry and distance tracking

### Telemetry & Monitoring
✓ Real-time web dashboard  
✓ MQTT data streaming  
✓ Live sensor visualizations  
✓ Success criteria checking  
✓ Event logging and history  
✓ Chart.js integration

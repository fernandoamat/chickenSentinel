# ChickenSentinel

An autonomous chicken deterrent system using computer vision and a water spray turret. The system detects birds and other animals using a DepthAI OAK-D stereo camera with YOLOv6 object detection, then tracks and sprays targets that venture outside designated "safe zones" (grass areas).

This project was develop in partnership with Gemini 3.0 to refine the ideas and parts list and then with Claude Code to implement all the code. It is definitely a passion project but it is saving my lawn and patio from being invaded by my chickens.

<img src="images/chickenSentinel_00%20Large.jpeg" alt="chickenSentinel00" width="400"><img src="images/chickenSentinel_01%20Large.jpeg" alt="chickenSentinel01" width="400">
Pictures of how the final system looks like. It is totally portable in a 12x12 wood board. The battery can easily be removed to charge. It last ~6-8 hours as long as it is not shooting all the time. Right image shows final build with the pump connected to bucket with water.


[Video](https://github.com/user-attachments/assets/673e242a-7802-41d2-aea9-533b167c6f6e) demonstrating the system detecting and squirting water to a bottle (capturing footage with the chickens was hard). It much easier to debug with inanimate objects but the system works exactly the same with any other object (like chickens) that can be detected and tracked by the camera.

## System Architecture

graph TD
    %% Nodes
    Battery[TalentCell Battery]
    Pi[Raspberry Pi Zero 2 W]
    Cam[Luxonis OAK-D Lite]
    Mosfet[MOSFET IRLZ44N]
    Pump[12V Water Pump]
    Servos[Pan/Tilt Servos]

    %% Styles
    style Battery fill:#f9f,stroke:#333,stroke-width:2px
    style Pi fill:#bbf,stroke:#333,stroke-width:2px
    style Cam fill:#bfb,stroke:#333,stroke-width:2px
    style Pump fill:#fbb,stroke:#333,stroke-width:2px

    %% Power Connections
    Battery -- "5V USB" --> Pi
    Battery -- "12V DC" --> Pump
    
    %% Data & Control Connections
    Pi -- "USB-C (Data)" --> Cam
    Pi -- "GPIO 17 (PWM)" --> Mosfet
    Pi -- "GPIO 12/13 (PWM)" --> Servos
    
    %% Electrical Logic
    Mosfet -- "Switches Ground" --> Pump
    Pi -- "GND (Common)" --> Battery
    Pi -- "5V Power" --> Servos


## Software design
```
┌─────────────────────┐     USB      ┌─────────────────────────────────┐
│      OAK-D          │─────────────▶│     Raspberry Pi Zero 2W        │
│  Stereo Camera      │              │                                 │
│                     │              │  ┌─────────────────────────┐    │
│  - Color camera     │              │  │  auto_turret_control.py │    │
│  - Left/Right mono  │              │  │                         │    │
│  - VPU (runs YOLO)  │              │  │  State Machine:         │    │
│                     │              │  │  SCANNING → TRACKING    │    │
│  Outputs:           │              │  │  TRACKING → SHOOTING    │    │
│  - Detections       │              │  │  SHOOTING → COOLDOWN    │    │
│  - Spatial coords   │              │  └───────────┬─────────────┘    │
│  - RGB frames       │              │              │                  │
└─────────────────────┘              │              ▼                  │
                                     │  ┌─────────────────────────┐    │
                                     │  │  Hardware Control       │    │
                                     │  │  - Pan servo (GPIO 12)  │    │
                                     │  │  - Tilt servo (GPIO 13) │    │
                                     │  │  - Water pump (GPIO 17) │    │
                                     │  └─────────────────────────┘    │
                                     └─────────────────────────────────┘
```

**Key insight:** YOLO inference runs on the OAK-D's neural accelerator (VPU), not on the Raspberry Pi. The Pi only receives detection results, making this efficient enough for the resource-constrained Zero 2W.

## Hardware Requirements

- **Camera:** OAK-D or OAK-D Lite (DepthAI-compatible stereo camera)
  - Color camera (CAM_A)
  - Left/Right mono cameras (CAM_B, CAM_C) for stereo depth
- **Controller:** Raspberry Pi Zero 2W (or any Pi with GPIO)
- **Actuators:**
  - 2x servo motors (pan/tilt) - standard 50Hz PWM servos
  - Water pump with relay/MOSFET control
- **Power:** Appropriate power supply for servos and pump

### GPIO Pin Assignments

| Component | GPIO Pin | PWM Channel |
|-----------|----------|-------------|
| Pan Servo | 12 | Channel 0 |
| Tilt Servo | 13 | Channel 1 |
| Water Pump | 17 | Digital Out |

## 🛠️ Parts lists
Total cost in December 2025 for the entire project was around $350.

| Component | Description | Model / Notes | Buy Link |
| :--- | :--- | :--- | :--- |
| **Controller** | Single Board Computer | **Raspberry Pi Zero 2 W**<br>The brain of the sentinel. | [Buy from Raspberry Pi](https://www.raspberrypi.com/products/raspberry-pi-zero-2-w/) |
| **Camera** | AI Camera | **Luxonis OAK-D Lite**<br>Planned upgrade for person/animal detection. | [Buy from Luxonis](https://shop.luxonis.com/products/oak-d-lite-1) |
| **Turret Mechanism** | Pan/Tilt Kit | **Pan/Tilt Bracket + 2x Servos**<br>Generic kit for aiming. | [Amazon](https://www.amazon.com/Yahboom-Pan-Tilt-Electric-Platform-Accessories/dp/B0BRXVFCKX) |
| **Water Cannon** | Pump | **12V DC Submersible Pump**<br>Connected to GPIO 17 via MOSFET. | [Amazon](https://www.amazon.com/dp/B00DLKT4OO) |
| **Pump Control** | Switch | **MOSFET Module DF-DFR0457**<br>Allows the 3.3V Pi to switch on/off the 12V pump. | [Buy from TME](https://www.tme.com/us/en-us/details/df-dfr0457/others-power-supply-modules/dfrobot/dfr0457/) |
| **Power Supply** | Battery | **TalentCell Rechargeable 12V/5V**<br>Powers both the Pi (via 5V USB) and Pump (via 12V DC). | [Amazon](https://www.amazon.com/dp/B01337QXMA) |
| **DC Buck Converter** | Converter | **Adjustable Power Supply**<br>Lowers the DC current of the battery from 12V to 7.6V for the servos. | [Amazon](https://www.amazon.com/dp/B078Q1624B) |
| **Power Splitter** | Cable Splitter | **Y-Splitter Cable**<br>Power splitter for both servos. | [Amazon](https://www.amazon.com/dp/B0FCD32RBB) |
| **USB Power Cable** | Power Cable | **USB to Barrel Jack**<br>Powers Raspberry Pi from battery. | [Amazon](https://www.amazon.com/dp/B0FS651N2G) |
| **DC Power Cables** | Power Cables | **DC Barrel Cables**<br>Connects splitter to DC converter. | [Amazon](https://www.amazon.com/dp/B0CRB1QP71) |
| **Vinyl Tubing** | Water Tubing | **Flexible Vinyl Tube**<br>Carries water from pump to nozzle. | [Amazon](https://www.amazon.com/dp/B07MTYMW13) |
| **USB-C Cable** | Data Cable | **USB-C to USB-A**<br>Connects OAK-D camera to Raspberry Pi. | [Amazon](https://www.amazon.com/dp/B0D97GS238) |
| **Jumper Wires** | Wiring | **Dupont Jumper Wires**<br>Pin wires to connect GPIO components. | [Amazon](https://www.amazon.com/dp/B01EV70C78) |

## File Structure

```
chickenSentinel/
├── chicken_tracker.py          # Desktop visualization with YOLO + depth
├── raspberrypi/
│   ├── auto_turret_control.py  # Autonomous turret controller
│   └── manual_turret_control.py # Manual servo/pump testing
├── notebooks/
│   └── debug_green_mask.ipynb  # Green zone detection debugging
├── openscad/                   # 3D printable parts
│   ├── nozzleAttachmentServo.scad
│   └── batteryBase.scad
├── pyproject.toml              # Project dependencies
├── CLAUDE.md                   # AI assistant instructions
└── README.md                   # This file
```

## Installation

```bash
# Clone the repository
git clone https://github.com/yourusername/chickenSentinel.git
cd chickenSentinel

# Install dependencies (uses uv package manager)
uv sync

# For Raspberry Pi, also install hardware PWM support:
pip install rpi-hardware-pwm gpiozero
```

## Usage

### Automated Turret (Production)

Run on the Raspberry Pi connected to the OAK-D camera:

```bash
uv run python raspberrypi/auto_turret_control.py
```

Monitor via SSH - the script outputs timestamped logs:
```
14:32:15 [INFO] Pipeline started. Entering main loop...
14:32:18 [INFO] Target acquired: bird conf=0.87 z=2450mm
14:32:19 [INFO] Centered! X=32mm. Shooting...
14:32:22 [INFO] Shot complete. Cooldown...
14:32:23 [INFO] Resuming tracking
14:32:26 [INFO] Target lost for 5.0s. Scanning...
14:32:45 [INFO] Status: state=SCANNING, pan=-15.0, tilt=0.0, detections=3
```

### Manual Turret Control (Testing)

For manual testing of servos and pump:

```bash
uv run python raspberrypi/manual_turret_control.py
```

Controls:
- Arrow keys: Pan/Tilt
- `s`: Shoot (3 seconds)
- `q`: Quit

### Desktop Visualization (Development)

For development/debugging with video display:

```bash
uv run python chicken_tracker.py
```

## Auto Turret Control - Detailed Documentation

### State Machine

The turret operates as a finite state machine with 4 states:

```
                    ┌──────────────────────────────────────┐
                    │                                      │
                    ▼                                      │
┌──────────┐   detection    ┌──────────┐   centered    ┌──────────┐
│ SCANNING │──────────────▶│ TRACKING │──────────────▶│ SHOOTING │
└──────────┘               └──────────┘               └──────────┘
     ▲                          │                          │
     │                          │                          │
     │      target lost         │                          │
     │      for 5 seconds       │                          │
     │◀─────────────────────────┤                          │
     │                                                     │
     │                     ┌──────────┐                    │
     │    cooldown done    │ COOLDOWN │◀───────────────────┘
     │◀────────────────────┴──────────┘    shoot done
```

| State | Description | Transitions |
|-------|-------------|-------------|
| `SCANNING` | No target detected. Pans back and forth within `SCAN_RANGE`. Tilt at neutral (0°). | → TRACKING (when valid target detected) |
| `TRACKING` | Target acquired. Adjusts pan/tilt to center target using trigonometry-based control. | → SHOOTING (when centered), → SCANNING (target lost for 5s) |
| `SHOOTING` | Pump activated. Holds position for `SHOOT_DURATION`. | → COOLDOWN (when done) |
| `COOLDOWN` | Pause after shooting (`COOLDOWN_DURATION`) or after full scan with no targets (`IDLE_COOLDOWN_DURATION`). | → TRACKING (if target visible), → SCANNING (if no target) |

### Configuration Constants

All tunable parameters are at the top of `raspberrypi/auto_turret_control.py`:

#### Detection Settings

| Constant | Default | Description |
|----------|---------|-------------|
| `ALLOWED_CLASSES` | `{'bird', 'horse', 'sheep', 'cow', 'bear', 'bottle'}` | YOLO classes to target |
| `FPS` | 15 | Camera frame rate (reduced for efficiency) |

#### Timing

| Constant | Default | Description |
|----------|---------|-------------|
| `SHOOT_DURATION` | 3.0 | Seconds to spray water |
| `COOLDOWN_DURATION` | 10.0 | Seconds pause after shooting |
| `IDLE_COOLDOWN_DURATION` | 30.0 | Seconds to idle after full scan with no detections |
| `GREEN_MASK_INTERVAL` | 10.0 | Seconds between green mask updates |

#### Scanning Mode

| Constant | Default | Description |
|----------|---------|-------------|
| `SCAN_RANGE` | `(-40, 60)` | Pan angle range in degrees |
| `SCAN_STEP` | 5 | Degrees per scan movement |
| `SCAN_DELAY` | 0.5 | Seconds between scan steps |
| `SCANNING_TILT_ANGLE` | 25.0 | Tilt angle during scanning (degrees) |

#### Tracking Control

| Constant | Default | Description |
|----------|---------|-------------|
| `CENTERING_THRESHOLD` | 50 | mm X-offset to consider "centered" |
| `TRACKING_DELAY` | 0.5 | Seconds to wait after servo command before next measurement |
| `TARGET_DETECTION_CONFIDENCE_THRESHOLD` | 0.5 | Minimum confidence to consider a detection valid |
| `TARGET_LOST_TIMEOUT` | 5.0 | Seconds before scanning after losing target |
| `MAX_PAN_RATE` | 2.0 | Maximum degrees per update cycle (smooth movement) |

#### Monitoring

| Constant | Default | Description |
|----------|---------|-------------|
| `STATUS_LOG_INTERVAL` | 30.0 | Seconds between periodic status logs |

### Key Functions

#### `calculate_target_y(z_mm: float) -> float`

**Location:** `auto_turret_control.py:102-118`

Calculates the target Y coordinate (in mm) that the turret should aim at based on target distance. The tilt servo uses proportional control to minimize the offset between the detection's actual Y coordinate and this target value. **This needs calibration** based on your turret mounting position and water nozzle trajectory.

Current implementation uses a linear regression formula:
```python
def calculate_target_y(z_mm: float) -> float:
    # y = 0.3654 * z - 282.318
    return (0.3654 * z_mm) - 282.318
```

Both pan and tilt use trigonometry-based angle calculations:
- **Pan**: Uses `atan2(x_offset, z_depth)` to calculate the exact angular offset to center the target horizontally
- **Tilt**: Uses `atan2(y_offset, z_depth)` where y_offset is the difference between actual Y and the calibrated target Y

#### `GreenZoneDetector`

**Location:** `auto_turret_control.py:126-174`

Detects grass/green areas using simple RGB thresholding. Targets standing on grass are considered "safe" and won't be sprayed.

- Updates mask every `GREEN_MASK_INTERVAL` seconds
- A pixel is "green" if: `G > R + threshold` AND `G > B + threshold`

#### `is_valid_spatial_coordinates(detection) -> bool`

**Location:** `auto_turret_control.py:182-199`

Validates that a detection has usable 3D coordinates:
- Checks for NaN values in x, y, z coordinates
- Ensures depth (z) is positive

#### `select_best_detection()`

**Location:** `auto_turret_control.py:202-244`

Selects the highest-confidence detection that:
1. Is in `ALLOWED_CLASSES`
2. Has valid spatial coordinates (not NaN, positive depth)
3. Is within 4000mm depth range
4. During scanning: has Y coordinate above -600mm

### Safety Features

The system will only shoot when ALL conditions are met:
1. Target class is in `ALLOWED_CLASSES`
2. Target has valid spatial coordinates (not NaN, positive depth)
3. Target depth is less than 4000mm
4. Target is centered (X offset < `CENTERING_THRESHOLD`)
5. Not in cooldown period

**Note:** Green zone detection (grass area filtering) is implemented but currently disabled in the code. A physical boundary was found to be more reliable than tuning the green detector.

### Robustness Features

The code includes several robustness improvements for reliable operation:

- **Non-blocking detection queue**: Uses `tryGet()` to prevent hanging if camera disconnects
- **Graceful degradation**: State machine continues running even without camera data
- **Hardware cleanup guarantee**: `finally` block ensures pump/servos are stopped on any exit
- **Exception handling**: All hardware operations wrapped in try/except with logging
- **Periodic status logging**: Logs state, angles, and detection count every 30 seconds
- **Pan rate limiting**: Prevents jerky servo movements with `MAX_PAN_RATE`
- **Tilt reset on scan**: Returns tilt to neutral when entering scanning mode


## Calibration Guide

### 1. Tracking Rate Tuning

The turret uses trigonometry-based angle calculations with `math.atan2()` to determine the exact angular offset to the target. The `MAX_PAN_RATE` constant limits how fast the servos can move per update cycle for smooth tracking.

If tracking is too slow, increase `MAX_PAN_RATE`. If the servos oscillate or overshoot, decrease it.

```python
MAX_PAN_RATE = 2.0  # Maximum degrees per update cycle
```

### 2. Target Y Calibration

The `calculate_target_y()` function defines what Y coordinate (in mm) the turret should aim at for a given depth. To calibrate:

1. Place a target at known distances (e.g., 1m, 2m, 3m, 4m, 5m)
2. For each distance, note the Y coordinate reported when the water hits the target
3. Fit a linear regression to your data points (Z vs Y)
4. Update the formula in `calculate_target_y()`

### 3. Green Zone Threshold

If the green detection is too sensitive (missing grass) or not sensitive enough (detecting non-grass as grass), adjust:

```python
# In GreenZoneDetector.__init__()
self.threshold = 0  # Increase if false positives, decrease if missing grass
```

## Dependencies

- `depthai>=3.2.1` - DepthAI camera SDK
- `opencv-python>=4.11.0.86` - Computer vision
- `numpy>=2.3.5` - Numerical processing
- `blobconverter>=1.4.3` - Neural network model conversion
- `matplotlib>=3.10.8` - Plotting (for debugging notebooks)
- `gpiozero>=2.0.1` - GPIO control (Raspberry Pi)
- `rpi-hardware-pwm` - Hardware PWM for servos (Raspberry Pi)

## 3D Printed Parts

The `openscad/` directory contains STL files for custom mounting hardware. All parts were designed in OpenSCAD and can be modified if needed.

| Part | File | Description |
| :--- | :--- | :--- |
| **Nozzle Adapter** | `nozzleAttachmentServo.stl` | Mounts the brass hose fitting to the pan/tilt bracket. Uses M3 screws. |
| **Battery Holder** | `batteryBase.stl` | Sleeve for the TalentCell battery with screw-down mounting flanges. |
| **Hose Connector** | `M0110_double-ended-barbed-hose-connector.stl` | Barbed connector for vinyl tubing. Can also be purchased online. |

**Print settings:** PLA or PETG, 0.2mm layer height, 20% infill. No supports needed.

## License

MIT License

# ChickenSentinel

An autonomous chicken deterrent system using computer vision and a water spray turret. The system detects birds and other animals using a DepthAI OAK-D stereo camera with YOLOv6 object detection, then tracks and sprays targets that venture outside designated "safe zones" (grass areas).

## Architecture

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
14:32:26 [INFO] Target lost for 2.0s. Scanning...
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
     │      for 2 seconds       │                          │
     │◀─────────────────────────┤                          │
     │                                                     │
     │                     ┌──────────┐                    │
     │    cooldown done    │ COOLDOWN │◀───────────────────┘
     │◀────────────────────┴──────────┘    shoot done
```

| State | Description | Transitions |
|-------|-------------|-------------|
| `SCANNING` | No target detected. Pans back and forth within `SCAN_RANGE`. Tilt at neutral (0°). | → TRACKING (when valid target detected) |
| `TRACKING` | Target acquired. Adjusts pan/tilt to center target using proportional control. | → SHOOTING (when centered), → SCANNING (target lost for 2s) |
| `SHOOTING` | Pump activated. Holds position for `SHOOT_DURATION`. | → COOLDOWN (when done) |
| `COOLDOWN` | Brief pause after shooting (`COOLDOWN_DURATION`). | → TRACKING (if target visible), → SCANNING (if no target) |

### Configuration Constants

All tunable parameters are at the top of `raspberrypi/auto_turret_control.py`:

#### Detection Settings

| Constant | Default | Description |
|----------|---------|-------------|
| `ALLOWED_CLASSES` | `{'bird', 'horse', 'sheep', 'cow', 'bear'}` | YOLO classes to target |
| `FPS` | 15 | Camera frame rate (reduced for efficiency) |

#### Timing

| Constant | Default | Description |
|----------|---------|-------------|
| `SHOOT_DURATION` | 3.0 | Seconds to spray water |
| `COOLDOWN_DURATION` | 1.0 | Seconds pause after shooting |
| `GREEN_MASK_INTERVAL` | 3.0 | Seconds between green zone updates |

#### Scanning Mode

| Constant | Default | Description |
|----------|---------|-------------|
| `SCAN_RANGE` | `(-60, 60)` | Pan angle range in degrees |
| `SCAN_STEP` | 5 | Degrees per scan movement |
| `SCAN_DELAY` | 0.5 | Seconds between scan steps |

#### Tracking Control

| Constant | Default | Description |
|----------|---------|-------------|
| `CENTERING_THRESHOLD` | 50 | mm X-offset to consider "centered" |
| `PAN_GAIN` | 0.05 | Proportional gain (degrees per mm) |
| `TARGET_LOST_TIMEOUT` | 2.0 | Seconds before scanning after losing target |
| `MAX_PAN_RATE` | 15.0 | Maximum degrees per cycle (smooth movement) |

#### Monitoring

| Constant | Default | Description |
|----------|---------|-------------|
| `STATUS_LOG_INTERVAL` | 30.0 | Seconds between periodic status logs |

### Key Functions

#### `calculate_tilt_from_depth(z_mm: float) -> float`

**Location:** `auto_turret_control.py:91-106`

Placeholder function that calculates tilt angle based on target distance. **This needs calibration** based on your actual water nozzle trajectory.

Current implementation:
- Close targets (1m): +10 degrees (aim slightly up)
- Far targets (5m): -20 degrees (compensate for water arc)

```python
def calculate_tilt_from_depth(z_mm: float) -> float:
    z_clamped = max(1000, min(5000, z_mm))
    tilt = 10 - (z_clamped - 1000) * 30 / 4000
    return tilt
```

#### `GreenZoneDetector`

**Location:** `auto_turret_control.py:113-162`

Detects grass/green areas using simple RGB thresholding. Targets standing on grass are considered "safe" and won't be sprayed.

- Updates mask every `GREEN_MASK_INTERVAL` seconds
- A pixel is "green" if: `G > R + threshold` AND `G > B + threshold`

#### `is_valid_spatial_coordinates(detection) -> bool`

**Location:** `auto_turret_control.py:169-187`

Validates that a detection has usable 3D coordinates:
- Checks for NaN values in x, y, z coordinates
- Ensures depth (z) is positive

#### `select_best_detection()`

**Location:** `auto_turret_control.py:190-217`

Selects the highest-confidence detection that:
1. Is in `ALLOWED_CLASSES`
2. Has valid spatial coordinates (not NaN, positive depth)
3. Is outside the green zone (not on grass)

### Safety Features

The system will only shoot when ALL conditions are met:
1. Target class is in `ALLOWED_CLASSES`
2. Target has valid spatial coordinates (not NaN, positive depth)
3. Target centroid is outside the green zone (grass)
4. Target is centered (X offset < `CENTERING_THRESHOLD`)
5. Not in cooldown period

### Robustness Features

The code includes several robustness improvements for reliable operation:

- **Non-blocking detection queue**: Uses `tryGet()` to prevent hanging if camera disconnects
- **Graceful degradation**: State machine continues running even without camera data
- **Hardware cleanup guarantee**: `finally` block ensures pump/servos are stopped on any exit
- **Exception handling**: All hardware operations wrapped in try/except with logging
- **Periodic status logging**: Logs state, angles, and detection count every 30 seconds
- **Pan rate limiting**: Prevents jerky servo movements with `MAX_PAN_RATE`
- **Tilt reset on scan**: Returns tilt to neutral when entering scanning mode

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

## Calibration Guide

### 1. Pan Gain Tuning

If tracking is too slow, increase `PAN_GAIN`. If it oscillates/overshoots, decrease it.

```python
PAN_GAIN = 0.05  # Start here, adjust as needed
```

### 2. Tilt/Depth Calibration

Test water trajectory at known distances and update `calculate_tilt_from_depth()`:

1. Place a target at 1m, 2m, 3m, 4m, 5m
2. Manually find the tilt angle that hits each distance
3. Update the function with your measured values

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
- `gpiozero>=2.0.1` - GPIO control (Raspberry Pi)
- `rpi-hardware-pwm` - Hardware PWM for servos (Raspberry Pi)

## License

MIT License

"""
Automated turret control for Raspberry Pi Zero 2W.

Combines DepthAI YOLOv6 detection with servo control for autonomous
chicken deterrent. YOLO inference runs on OAK-D's VPU (neural accelerator),
so the RPi only handles lightweight control logic.

Usage:
    uv run python raspberrypi/auto_turret_control.py
"""

import logging
import math
import time
from enum import Enum

import depthai as dai
import numpy as np
from gpiozero import DigitalOutputDevice
from rpi_hardware_pwm import HardwarePWM

# =============================================================================
# CONFIGURATION - Adjust these constants as needed
# =============================================================================

# Detection settings
ALLOWED_CLASSES = {'bird', 'horse', 'sheep', 'cow', 'bear', 'bottle'}
FPS = 15  # Reduced from 30 for RPi Zero 2W efficiency

# Timing
SHOOT_DURATION = 3.0  # Seconds to spray water
COOLDOWN_DURATION = 10.0  # Seconds after shooting before resuming
IDLE_COOLDOWN_DURATION = 30.0  # Seconds to idle after full scan with no detections. Increase to preserve battery.
GREEN_MASK_INTERVAL = 10.0  # Seconds between green mask updates. Increase to reduce CPU load.

# Scanning mode
SCAN_RANGE = (-40, 60)  # Pan angle range (degrees)
SCAN_STEP = 5  # Degrees per scan movement
SCAN_DELAY = 0.5  # Seconds between scan steps
SCANNING_TILT_ANGLE = 25.0  # Tilt angle during scanning (degrees). Adjust to your mounting position.

# Tracking control
CENTERING_THRESHOLD = 50  # mm - X offset considered "centered"
TRACKING_DELAY = 0.5  # Seconds to wait after servo command before next measurement

# Targeting control
TARGET_DETECTION_CONFIDENCE_THRESHOLD = 0.5  # Minimum confidence to consider a detection valid. Increase to reduce false positives.

# Hardware pins
PUMP_PIN = 17
PAN_PWM_CHANNEL = 0  # GPIO 12
TILT_PWM_CHANNEL = 1  # GPIO 13

# Tracking behavior
TARGET_LOST_TIMEOUT = 5.0  # Seconds before scanning after losing target
MAX_PAN_RATE = 2.0  # Maximum degrees per update cycle for smooth movement. Critical for smooth tracking.

# Monitoring
STATUS_LOG_INTERVAL = 30.0  # Seconds between periodic status logs

# =============================================================================
# HARDWARE SETUP
# =============================================================================


def set_angle(pwm_device, angle: float, log) -> None:
    """
    Set servo angle.

    Args:
        pwm_device: HardwarePWM instance
        angle: Angle in degrees (-90 to +90)
    """
    # too much logging
    # log.info(f"Setting servo angle (pan or tilt) to {angle} degrees")
    angle = max(-90, min(90, angle))
    # Convert angle to duty cycle (Equation: 2.5 + (angle + 90) / 180 * 10)
    # Servo PWM: 2.5% = -90deg, 7.5% = 0deg, 12.5% = +90deg
    duty_cycle = 2.5 + (angle + 90) / 18.0
    pwm_device.change_duty_cycle(duty_cycle)


# =============================================================================
# STATE MACHINE
# =============================================================================


class TurretState(Enum):
    SCANNING = 1  # No detection, panning to search
    TRACKING = 2  # Detection found, centering on target
    SHOOTING = 3  # Centered and outside green zone, firing
    COOLDOWN = 4  # Just shot, brief pause before next action


# =============================================================================
# TARGET Y CALCULATION
# You need to change this method based on your own calibration data
# It depends on where you mount the turret
# =============================================================================


def calculate_target_y(z_mm: float) -> float:
    """
    Calculate target Y coordinate (mm) based on target distance (Z).

    The turret should aim to minimize the offset between the detection's
    actual Y coordinate and this target Y value.

    Args:
        z_mm: Distance to target in millimeters

    Returns:
        Target Y coordinate in millimeters

    """
    # Linear regression formula: y = 0.3654 * z - 282.318
    # You need to change this based on your own calibration data
    return (0.3654 * z_mm) - 282.318


# =============================================================================
# GREEN ZONE DETECTION
# =============================================================================


class GreenZoneDetector:
    """
    Detects green zones (grass) in camera frames.

    Animals in green zones are considered safe and won't be targeted.
    Uses simple RGB threshold: a pixel is "green" if G > R and G > B.
    """

    def __init__(self, threshold: int = 0, update_interval: float = GREEN_MASK_INTERVAL):
        self.threshold = threshold
        self.update_interval = update_interval
        self.last_mask = None
        self.last_update = 0.0

    def update_mask(self, rgb_frame: np.ndarray) -> None:
        """Update green mask if interval has elapsed."""
        now = time.time()
        if self.last_mask is None or (now - self.last_update) >= self.update_interval:
            # Green channel must exceed both R and B by threshold
            g = rgb_frame[:, :, 1].astype(np.int16)
            self.last_mask = (
                ((g - rgb_frame[:, :, 2]) > self.threshold) &
                ((g - rgb_frame[:, :, 0]) > self.threshold)
            )
            self.last_update = now

    def is_outside_green(self, detection) -> bool:
        """
        Check if detection centroid is outside the green zone.

        Args:
            detection: Detection object with xmin, xmax, ymin, ymax (normalized 0-1)

        Returns:
            True if centroid is on a non-green pixel, False otherwise.
        """
        if self.last_mask is None:
            return False

        h, w = self.last_mask.shape
        cx = int(((detection.xmin + detection.xmax) / 2) * w)
        cy = int(((detection.ymin + detection.ymax) / 2) * h)

        # Clamp to valid coordinates
        cx = max(0, min(cx, w - 1))
        cy = max(0, min(cy, h - 1))

        # Outside green = mask is False at that pixel
        return not self.last_mask[cy, cx]


# =============================================================================
# DETECTION SELECTION
# =============================================================================


def is_valid_spatial_coordinates(detection) -> bool:
    """
    Check if detection has valid spatial coordinates.

    Args:
        detection: Detection object with spatialCoordinates

    Returns:
        True if coordinates are valid (non-zero depth, not NaN).
    """
    coords = detection.spatialCoordinates
    # Check for NaN
    if coords.x != coords.x or coords.y != coords.y or coords.z != coords.z:
        return False
    # Check for zero/invalid depth
    if coords.z <= 0:
        return False
    return True


def select_best_detection(detections, allowed_classes: set,
                          green_detector: GreenZoneDetector, log, is_scanning: bool = False):
    """
    Select the highest-confidence detection from allowed classes outside green zone.

    Args:
        detections: List of detection objects
        allowed_classes: Set of class names to consider
        green_detector: GreenZoneDetector instance
        is_scanning: If True, apply stricter y_nm filtering

    Returns:
        Best detection or None if no valid detection found.
    """
    valid = []
    for det in detections:
        #log.info(f"Select best detection: processing {det.labelName}")
        if det.labelName not in allowed_classes:
            continue
        #log.info("Before is_valid_spatial_coordinates")
        if not is_valid_spatial_coordinates(det):
            continue
        # Filter by depth and height boundaries
        # The target must be within reasonable distance (z_nm low)
        # The target has to be on the lower part of the frame (y_nm high). So even when shooting that will be the case
        z_nm = det.spatialCoordinates.z
        y_nm = det.spatialCoordinates.y
        if z_nm >= 4000:
            continue
        elif is_scanning and y_nm <= -600:
            continue
        #log.info("Before green_detector")
        # it was easier to setup a physical boundary than to tune the green detector
        #if not green_detector.is_outside_green(det):
        #    continue
        log.info(f"Adding {det.labelName} to valid detections. Coordinates: x={det.spatialCoordinates.x:.2f}, y={det.spatialCoordinates.y:.2f}, z={det.spatialCoordinates.z:.2f} Passed all checks")
        valid.append(det)

    if not valid:
        return None

    # Return detection with highest confidence
    return max(valid, key=lambda d: d.confidence)


# =============================================================================
# MAIN CONTROL LOOP
# =============================================================================


def main():
    # Setup logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(message)s',
        datefmt='%H:%M:%S'
    )
    log = logging.getLogger('turret')

    # Initialize hardware (done here, not at module level, for better error handling)
    log.info("Initializing hardware...")
    try:
        pan_servo = HardwarePWM(pwm_channel=PAN_PWM_CHANNEL, hz=50)
        tilt_servo = HardwarePWM(pwm_channel=TILT_PWM_CHANNEL, hz=50)
        pump = DigitalOutputDevice(PUMP_PIN)
    except Exception as e:
        log.error(f"Failed to initialize hardware: {e}")
        log.error("Make sure you're running on a Raspberry Pi with proper GPIO access.")
        return

    try:
        pan_servo.start(0)
        tilt_servo.start(0)
    except Exception as e:
        log.error(f"Failed to start servos: {e}")
        return

    # State variables
    state = TurretState.SCANNING
    pan_angle = 0.0
    tilt_angle = SCANNING_TILT_ANGLE  # start upright
    last_target_seen_time = time.time()  # Tracks when we last saw a target (for faster scan resumption)
    scan_direction = 1  # 1 = moving toward positive, -1 = toward negative
    shoot_start = 0.0
    cooldown_start = 0.0
    last_status_log = time.time()  # For periodic status logging
    detection_count = 0  # Count valid detections for status logging
    scan_detected_target = False  # Track if target seen during current scan sweep
    is_idle_cooldown = False  # Distinguish idle vs post-shoot cooldown

    green_detector = GreenZoneDetector()

    log.info("Initializing DepthAI pipeline...")

    # Wrap everything in try/finally to ensure hardware cleanup even if pipeline setup fails
    try:
        # Build DepthAI pipeline
        with dai.Pipeline() as p:
            # Camera setup
            camRgb = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
            monoLeft = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
            monoRight = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
            stereo = p.create(dai.node.StereoDepth)

            # Spatial detection network (YOLO runs on OAK-D VPU)
            modelDesc = dai.NNModelDescription("yolov6-nano")
            spatialNet = p.create(dai.node.SpatialDetectionNetwork).build(
                camRgb, stereo, modelDesc, fps=FPS
            )

            # Configure stereo depth
            stereo.setExtendedDisparity(True)
            platform = p.getDefaultDevice().getPlatform()
            if platform == dai.Platform.RVC2:
                stereo.setOutputSize(640, 400)

            # Configure spatial detection network
            spatialNet.input.setBlocking(False)
            spatialNet.setBoundingBoxScaleFactor(0.5)
            spatialNet.setDepthLowerThreshold(100)
            spatialNet.setDepthUpperThreshold(10000)
            spatialNet.setConfidenceThreshold(TARGET_DETECTION_CONFIDENCE_THRESHOLD)

            # Link mono cameras to stereo
            monoLeft.requestOutput((640, 400)).link(stereo.left)
            monoRight.requestOutput((640, 400)).link(stereo.right)

            # Create output queues (no display needed - headless operation)
            detQueue = spatialNet.out.createOutputQueue()
            rgbQueue = spatialNet.passthrough.createOutputQueue(maxSize=1, blocking=False)

            p.start()
            log.info("Pipeline started. Entering main loop...")

            # Set initial servo positions
            try:
                set_angle(pan_servo, pan_angle, log)
                set_angle(tilt_servo, tilt_angle, log)
            except Exception as e:
                log.error(f"Failed to set initial servo positions: {e}")

            while p.isRunning():
                now = time.time()

                # Periodic status logging for monitoring (always runs)
                if now - last_status_log >= STATUS_LOG_INTERVAL:
                    log.info(
                        f"Status: state={state.name}, pan={pan_angle:.1f}, "
                        f"tilt={tilt_angle:.1f}, detections={detection_count}"
                    )
                    last_status_log = now
                    detection_count = 0

                # Get detections (non-blocking to prevent hanging if camera disconnects)
                det_msg = detQueue.tryGet()

                # Get RGB frame for green mask (non-blocking)
                rgb_msg = rgbQueue.tryGet()
                if rgb_msg is not None:
                    green_detector.update_mask(rgb_msg.getCvFrame())

                # Find best target (only if we have detections)
                target = None
                if det_msg is not None:
                    detections = det_msg.detections
                    target = select_best_detection(
                        detections, ALLOWED_CLASSES, green_detector, log,
                        is_scanning=(state == TurretState.SCANNING)
                    )
                # uncommenting this generates too much log info
                #else:
                #    log.info("Debugging:det_msg is none")

                if target is not None:
                    last_target_seen_time = now
                    detection_count += 1
                    scan_detected_target = True
                    x_offset = target.spatialCoordinates.x  # mm from center
                    z_depth = target.spatialCoordinates.z

                    # Transition from scanning to tracking
                    if state == TurretState.SCANNING:
                        state = TurretState.TRACKING
                        log.info(
                            f"Target acquired: {target.labelName} "
                            f"conf={target.confidence:.2f} z={z_depth:.2f}mm x={x_offset:.2f}mm from center"
                        )

                    if state == TurretState.TRACKING:
                        # Calculate angle to target using trigonometry (not mm-based gain)
                        # This gives the actual angular offset from camera center
                        # NOTE: Pan direction sign may need to be flipped depending on
                        # how your camera and servo are mounted. Test and adjust if needed.
                        angle_to_target = math.degrees(math.atan2(x_offset, z_depth))
                        pan_adjustment = -angle_to_target

                        # Rate limiting: clamp adjustment to MAX_PAN_RATE per cycle
                        pan_adjustment = max(-MAX_PAN_RATE, min(MAX_PAN_RATE, pan_adjustment))
                        pan_angle = max(-90, min(90, pan_angle + pan_adjustment))

                        # Calculate tilt angle using trigonometry (same approach as pan)
                        # Calculate Y offset from target position
                        y_actual = target.spatialCoordinates.y
                        y_target = calculate_target_y(z_depth)
                        y_offset = y_actual - y_target

                        angle_to_target_y = math.degrees(math.atan2(y_offset, z_depth))
                        tilt_adjustment = -angle_to_target_y
                        tilt_adjustment = max(-MAX_PAN_RATE, min(MAX_PAN_RATE, tilt_adjustment))
                        tilt_angle = max(-90, min(90, tilt_angle + tilt_adjustment))

                        try:
                            log.info(
                                f"Tracking: "
                                f"x_offset={x_offset:.1f}mm, angle_x={angle_to_target:.2f}deg, "
                                f"pan_adj={pan_adjustment:.2f}, pan={pan_angle:.2f}; "
                                f"y_offset={y_offset:.1f}mm, angle_y={angle_to_target_y:.2f}deg, "
                                f"tilt_adj={tilt_adjustment:.2f}, tilt={tilt_angle:.2f}"
                            )
                            set_angle(pan_servo, pan_angle, log)
                            set_angle(tilt_servo, tilt_angle, log)
                            # Wait for servo to physically move before next measurement
                            # We are tracking slow moving animals, so this delay is acceptable and adds stability
                            time.sleep(TRACKING_DELAY)
                        except Exception as e:
                            log.error(f"Servo error: {e}")

                        # Check if centered enough to shoot
                        if abs(x_offset) < CENTERING_THRESHOLD:
                            log.info(f"Centered! X={x_offset:.0f}mm. Shooting...")
                            state = TurretState.SHOOTING
                            try:
                                pump.on()
                            except Exception as e:
                                log.error(f"Pump error: {e}")
                            shoot_start = now

                    elif state == TurretState.SHOOTING:
                        # Hold position, wait for shoot duration
                        if now - shoot_start >= SHOOT_DURATION:
                            try:
                                log.info("Debugging: Deactivating pump...")
                                pump.off()
                            except Exception as e:
                                log.error(f"Pump error: {e}")
                            log.info("Shot complete. Cooldown...")
                            state = TurretState.COOLDOWN
                            cooldown_start = now
                            is_idle_cooldown = False

                    elif state == TurretState.COOLDOWN:
                        cooldown_duration = IDLE_COOLDOWN_DURATION if is_idle_cooldown else COOLDOWN_DURATION
                        if now - cooldown_start >= cooldown_duration:
                            state = TurretState.TRACKING
                            log.info("Resuming tracking")

                else:
                    # No valid detection
                    previous_state = state

                    if state == TurretState.SHOOTING:
                        # Finish shooting even if target lost
                        if now - shoot_start >= SHOOT_DURATION:
                            try:
                                pump.off()
                            except Exception as e:
                                log.error(f"Pump error: {e}")
                            log.info("Shot complete (target lost). Scanning...")
                            state = TurretState.SCANNING

                    elif state == TurretState.COOLDOWN:
                        cooldown_duration = IDLE_COOLDOWN_DURATION if is_idle_cooldown else COOLDOWN_DURATION
                        if now - cooldown_start >= cooldown_duration:
                            log.info("Cooldown complete. Scanning...")
                            state = TurretState.SCANNING

                    elif state == TurretState.TRACKING:
                        # Target lost while tracking - use shorter timeout for faster response
                        if now - last_target_seen_time > TARGET_LOST_TIMEOUT:
                            log.info(f"Target lost for {TARGET_LOST_TIMEOUT}s. Scanning...")
                            state = TurretState.SCANNING

                    # Reset tilt to neutral when transitioning to SCANNING
                    if state == TurretState.SCANNING and previous_state != TurretState.SCANNING:
                        tilt_angle = SCANNING_TILT_ANGLE
                        try:
                            set_angle(tilt_servo, tilt_angle, log)
                        except Exception as e:
                            log.error(f"Servo error resetting tilt: {e}")

                    # Execute scanning behavior when in SCANNING state
                    if state == TurretState.SCANNING:
                        # Scan: pan back and forth
                        pan_angle += scan_direction * SCAN_STEP
                        if pan_angle >= SCAN_RANGE[1]:
                            pan_angle = SCAN_RANGE[1]
                            scan_direction = -1
                            if not scan_detected_target:
                                state = TurretState.COOLDOWN
                                cooldown_start = now
                                is_idle_cooldown = True
                                log.info("Full scan complete, no targets. Entering idle cooldown...")
                            scan_detected_target = False
                        elif pan_angle <= SCAN_RANGE[0]:
                            pan_angle = SCAN_RANGE[0]
                            scan_direction = 1
                            if not scan_detected_target:
                                state = TurretState.COOLDOWN
                                cooldown_start = now
                                is_idle_cooldown = True
                                log.info("Full scan complete, no targets. Entering idle cooldown...")
                            scan_detected_target = False

                        try:
                            set_angle(pan_servo, pan_angle, log)
                        except Exception as e:
                            log.error(f"Servo error: {e}")
                        time.sleep(SCAN_DELAY)

                # Small sleep to reduce CPU usage
                time.sleep(0.01)

    except KeyboardInterrupt:
        log.info("Interrupted by user. Shutting down...")
    except Exception as e:
        log.error(f"Pipeline error: {e}")
    finally:
        # Ensure hardware is safely shut down (runs even if pipeline setup fails)
        try:
            pump.off()
        except Exception as e:
            log.error(f"Error turning off pump: {e}")
        try:
            pan_servo.stop()
            tilt_servo.stop()
        except Exception as e:
            log.error(f"Error stopping servos: {e}")
        log.info("Cleanup complete.")


if __name__ == "__main__":
    main()

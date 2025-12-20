# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ChickenSentinel is a real-time chicken detection system using DepthAI stereo cameras. It uses computer vision models for object detections (e.g. MobileNet-SSD) to detect birds and other animals in the camera feed and provides spatial coordinates (X, Y, Z in millimeters) for detected animals.

## Commands

```bash
# Install dependencies (uses uv package manager)
uv sync

# Run the chicken tracker
uv run python chicken_tracker_v0.py
```

## Architecture

The system uses DepthAI's pipeline architecture:
1. **ColorCamera (camRgb)** - Captures 1080P color video, outputs 300x300 preview for AI
2. **MonoCameras (left/right)** - Two 400P mono cameras for stereo depth calculation
3. **StereoDepth** - Computes depth map from the stereo pair
4. **MobileNetSpatialDetectionNetwork** - Runs MobileNet-SSD inference and fuses with depth data to provide 3D coordinates

The main detection loop filters for "bird" class (label ID 3) from MobileNet-SSD and displays bounding boxes with spatial coordinates.

## Hardware Requirements

- OAK-D or other DepthAI-compatible stereo camera
- The camera must have left/right mono cameras (CAM_B, CAM_C) and a color camera (CAM_A)

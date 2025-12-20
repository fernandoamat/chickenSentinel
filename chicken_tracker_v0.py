import cv2
import depthai as dai
import numpy as np

"""
Chicken Tracker v0
This script uses DepthAI to detect chickens in real-time using a stereo camera setup.
It displays the detected chicken with bounding boxes and spatial coordinates.

Main steps:
- Read color video.
- Calculate Depth (using the left/right cameras).
- Run AI to find "birds".
- Fuse the AI + Depth to give you the exact X, Y, Z coordinates of the chicken relative to the camera.

Using Yolo v6-nano for detection since it is "natively" supported by DepthAI v3.x

# TODO: expand to other animals and improve accuracy.
# TODO: add a mask to detect allowed boundaries or not allowed boundaries.

DepthAI V3 API Reference: https://docs.luxonis.com/software-v3/depthai/tutorials/v2-vs-v3/
"""

# --- 1. Define the Pipeline (V3 API) ---
with dai.Pipeline() as pipeline:
    # Create cameras using V3 .build() syntax
    camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)

    # Create stereo depth with .build() - use smaller resolution to save shaves for NN
    stereo = pipeline.create(dai.node.StereoDepth).build(
        autoCreateCameras=True,
        presetMode=dai.node.StereoDepth.PresetMode.DEFAULT,
        size=(640, 400)
    )
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    # Reduce stereo post-processing to free shaves for neural network
    stereo.setPostProcessingHardwareResources(0, 0)

    # Create spatial detection network with YOLOv6-nano (V3 API requires YOLO models)
    spatialDetectionNetwork = pipeline.create(dai.node.SpatialDetectionNetwork).build(
        camRgb, stereo, dai.NNModelDescription("yolov6-nano")
    )
    spatialDetectionNetwork.setConfidenceThreshold(0.5)
    spatialDetectionNetwork.input.setBlocking(False)
    spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
    spatialDetectionNetwork.setDepthLowerThreshold(100)
    spatialDetectionNetwork.setDepthUpperThreshold(5000)

    # Get label map from the model (COCO classes)
    labelMap = spatialDetectionNetwork.getClasses()

    # Create output queues (V3: no XLinkOut nodes needed)
    previewOutput = camRgb.requestOutput((300, 300), dai.ImgFrame.Type.BGR888p)
    previewQueue = previewOutput.createOutputQueue()
    detectionQueue = spatialDetectionNetwork.out.createOutputQueue()

    # Start the pipeline
    pipeline.start()

    print("Chicken Tracker Started! Press 'q' to quit.")
    print(f"Available classes: {labelMap}")

    # --- 2. Main Loop ---
    while pipeline.isRunning():
        inPreview = previewQueue.get()
        inDet = detectionQueue.get()

        frame = inPreview.getCvFrame()
        detections = inDet.detections

        for detection in detections:
            # Check if the detected object is a 'bird' (COCO class)
            # TODO: Add more labels for other animals or objects or for testing purposes.
            label = labelMap[detection.label] if detection.label < len(labelMap) else "unknown"
            if label == "bird" or label == "tv":  # COCO uses "tv" not "tvmonitor"
                # Get Coordinates (in millimeters)
                x = detection.spatialCoordinates.x
                y = detection.spatialCoordinates.y
                z = detection.spatialCoordinates.z

                # Formatting for display
                text = f"Chicken: X:{int(x)}mm Y:{int(y)}mm Z:{int(z)}mm"

                # Draw Box and Text
                x1 = int(detection.xmin * 300)
                y1 = int(detection.ymin * 300)
                x2 = int(detection.xmax * 300)
                y2 = int(detection.ymax * 300)

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Print to terminal for debugging
                print(f"FOUND CHICKEN! Coordinates -> X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f}")

        cv2.imshow("Chicken Tracker View", frame)

        if cv2.waitKey(1) == ord('q'):
            break

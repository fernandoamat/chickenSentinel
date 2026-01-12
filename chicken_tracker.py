from pathlib import Path
import sys
import cv2
import depthai as dai
import numpy as np
import time

"""
Example from https://docs.luxonis.com/software-v3/depthai/examples/spatial_detection_network/spatial_detection/
For Depth AI v3.x API

Uses the YOLOv6-nano model for object detection and spatial information.
"""

modelDescription = dai.NNModelDescription("yolov6-nano")
FPS = 15

class SpatialVisualizer(dai.node.HostNode):
    def __init__(self):
        dai.node.HostNode.__init__(self)
        self.sendProcessingToPipeline(True)
        self.lastGreenMask = None
        self.lastGreenMaskTime = 0
        self.greenMaskInterval = 3.0  # seconds between mask updates
        self.greenThreshold = 0  # minimum difference G must exceed R and B by
        self.allowedClasses = {'bird', 'horse', 'sheep', 'cow', 'bear', 'bottle'}  # Set of allowed class names (None = allow all)

    def setAllowedClasses(self, classes):
        """
        Set the allowed YOLO classes for detection display.

        Args:
            classes: Iterable of class names to allow (e.g., {'bird', 'cat', 'dog'})
                     Pass None to allow all classes.
        """
        self.allowedClasses = set(classes) if classes is not None else None

    def isAllowedDetection(self, detection):
        """
        Check if a detection's class is in the allowed set.

        Args:
            detection: A detection object with labelName attribute

        Returns:
            True if detection should be displayed, False otherwise
        """
        if self.allowedClasses is None:
            return True
        return detection.labelName in self.allowedClasses

    def isOutsideGreenZone(self, detection):
        """
        Check if a detection's centroid is outside the green zone (on a pixel with value 0).

        Args:
            detection: A detection object with xmin, xmax, ymin, ymax (normalized 0-1)

        Returns:
            True if centroid is on a non-green pixel (value 0), False otherwise.
            Returns False if no green mask is available yet.
        """
        if self.lastGreenMask is None:
            return False

        # TODO: is cnetroid what we want? Or should we use lowest corner of the bounding box?
        # play around once the enrtire system is setup
        height, width = self.lastGreenMask.shape[:2]
        cx = int(((detection.xmin + detection.xmax) / 2) * width)
        cy = int(((detection.ymin + detection.ymax) / 2) * height)

        # Clamp to valid pixel coordinates
        cx = max(0, min(cx, width - 1))
        cy = max(0, min(cy, height - 1))

        return self.lastGreenMask[cy, cx] == 0

    def build(self, depth:dai.Node.Output, detections: dai.Node.Output, rgb: dai.Node.Output):
        self.link_args(depth, detections, rgb) # Must match the inputs to the process method

    def process(self, depthPreview, detections, rgbPreview):
        depthPreview = depthPreview.getCvFrame()
        rgbPreview = rgbPreview.getCvFrame()
        depthFrameColor = self.processDepthFrame(depthPreview)

        # Only recalculate green mask every greenMaskInterval seconds
        currentTime = time.time()
        if self.lastGreenMask is None or (currentTime - self.lastGreenMaskTime) >= self.greenMaskInterval:
            self.lastGreenMask = self.processRGBFrame(rgbPreview, green_threshold=self.greenThreshold)
            self.lastGreenMaskTime = currentTime

        self.displayResults(rgbPreview, depthFrameColor, self.lastGreenMask, detections.detections)

    def processDepthFrame(self, depthFrame):
        depth_downscaled = depthFrame[::4]
        if np.all(depth_downscaled == 0):
            min_depth = 0
        else:
            min_depth = np.percentile(depth_downscaled[depth_downscaled != 0], 1)
        max_depth = np.percentile(depth_downscaled, 99)
        depthFrameColor = np.interp(depthFrame, (min_depth, max_depth), (0, 255)).astype(np.uint8)
        return cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)

    def processRGBFrame(self, rgbFrame, green_threshold=0):
        """
        Returns a binary mask of pixels with a high green component.
        This is an easy way to detect grass areas.
        Feel free to replace this with a more sophisticated method.

        A pixel is considered "green" if its green channel exceeds both
        red and blue channels by at least green_threshold.

        Args:
            rgbFrame: BGR image from OpenCV
            green_threshold: minimum difference G must exceed R and B by

        Returns:
            Binary mask (uint8) where 255 = green pixel, 0 = non-green
            The mask is a binary image where 255 represents a green pixel and 0 represents a non-green pixel.
            Thus, animals can stay in the green areas, and the mask can be used to track their movements.
        """
        
        """
        # Debugging:Save frame for offline debugging (exact values preserved)
        # uncomment this to save the frame for debugging.
        # then user notebooks/debug_green_mask.py to visualize the saved frames.
        timestamp = int(time.time() * 1000)
        np.save(f"tmp/rgb_{timestamp}.npy", rgbFrame)
        print(f'Saved frame at tmp/rgb_{timestamp}.npy with shape {rgbFrame.shape} and green_threshold {green_threshold}')
        """

        # Use array slicing (views) instead of cv2.split (copies)
        # Only convert green channel to int16 to avoid underflow during subtraction
        g_int = rgbFrame[:, :, 1].astype(np.int16)
        green_mask = ((g_int - rgbFrame[:, :, 2]) > green_threshold) & \
                     ((g_int - rgbFrame[:, :, 0]) > green_threshold)
        return (green_mask * 255).astype(np.uint8)

    def displayResults(self, rgbFrame, depthFrameColor, greenMask, detections):
        height, width, _ = rgbFrame.shape
        for detection in detections:
            if not self.isAllowedDetection(detection):
                continue
            if not self.isOutsideGreenZone(detection):
                continue
            self.drawBoundingBoxes(depthFrameColor, detection)
            self.drawDetections(rgbFrame, detection, width, height)

        cv2.imshow("depth", depthFrameColor)
        cv2.imshow("rgb", rgbFrame)
        cv2.imshow("green_mask", greenMask)
        if cv2.waitKey(1) == ord('q'):
            self.stopPipeline()

    def drawBoundingBoxes(self, depthFrameColor, detection):
        roiData = detection.boundingBoxMapping
        roi = roiData.roi
        roi = roi.denormalize(depthFrameColor.shape[1], depthFrameColor.shape[0])
        topLeft = roi.topLeft()
        bottomRight = roi.bottomRight()
        cv2.rectangle(depthFrameColor, (int(topLeft.x), int(topLeft.y)), (int(bottomRight.x), int(bottomRight.y)), (255, 255, 255), 1)

    def drawDetections(self, frame, detection, frameWidth, frameHeight):
        x1 = int(detection.xmin * frameWidth)
        x2 = int(detection.xmax * frameWidth)
        y1 = int(detection.ymin * frameHeight)
        y2 = int(detection.ymax * frameHeight)
        label = detection.labelName
        color = (255, 255, 255)
        cv2.putText(frame, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
        cv2.putText(frame, "{:.2f}".format(detection.confidence * 100), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
        cv2.putText(frame, f"X: {int(detection.spatialCoordinates.x)} mm", (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
        cv2.putText(frame, f"Y: {int(detection.spatialCoordinates.y)} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
        cv2.putText(frame, f"Z: {int(detection.spatialCoordinates.z)} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 1)

# Creates the pipeline and a default device implicitly
with dai.Pipeline() as p:
    # Define sources and outputs
    camRgb = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    monoLeft = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    monoRight = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    stereo = p.create(dai.node.StereoDepth)
    spatialDetectionNetwork = p.create(dai.node.SpatialDetectionNetwork).build(camRgb, stereo, modelDescription, fps=FPS)
    visualizer = p.create(SpatialVisualizer)

    # setting node configs
    stereo.setExtendedDisparity(True)
    platform = p.getDefaultDevice().getPlatform()
    if platform == dai.Platform.RVC2:
        # For RVC2, width must be divisible by 16
        stereo.setOutputSize(640, 400)

    spatialDetectionNetwork.input.setBlocking(False)
    spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
    spatialDetectionNetwork.setDepthLowerThreshold(100)
    spatialDetectionNetwork.setDepthUpperThreshold(5000)
    spatialDetectionNetwork.setConfidenceThreshold(0.25)

    # Linking
    monoLeft.requestOutput((640, 400)).link(stereo.left)
    monoRight.requestOutput((640, 400)).link(stereo.right)

    visualizer.build(spatialDetectionNetwork.passthroughDepth, spatialDetectionNetwork.out, spatialDetectionNetwork.passthrough)

    p.run()

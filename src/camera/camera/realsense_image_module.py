#!/usr/bin/env python3
"""
RealSense camera image capture module for acquiring RGB-D frames.
"""

import cv2
import numpy as np
import pyrealsense2 as rs


class RealSenseImage:
    """RealSense camera image capture class."""

    def __init__(self, SN_number=None, width=640, height=480, fps=30, enable_depth=True):
        """
        Initialize the RealSense camera.

        Args:
            SN_number: camera serial number; if None, the default device is used.
            width: image width.
            height: image height.
            fps: frame rate.
            enable_depth: whether to enable the depth stream.
        """
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.enable_depth = enable_depth
        
        if SN_number:
            self.config.enable_device(SN_number)

        # Enable depth only when needed to reduce bandwidth in RGB-only mode.
        self.config.enable_stream(rs.stream.color, width, height, rs.format.rgb8, fps)
        if self.enable_depth:
            self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)

        self.pipeline.start(self.config)

        # Aligner maps the depth frame onto the color frame.
        self.align_to = rs.stream.color
        self.aligner = rs.align(self.align_to) if self.enable_depth else None

        self.depth_image_np = None
        self.color_image_np = None

        self._stabilize_camera()

    def _stabilize_camera(self):
        """Discard the first few frames to let the camera output stabilize."""
        for _ in range(10):
            if self.enable_depth:
                self.capture_rgb_depth_frames()
            else:
                self.capture_rgb_frame()
    
    def capture_rgb_depth_frames(self):
        """
        Capture RGB and depth frames together.

        Returns:
            tuple: (rgb_image, depth_image)
                - rgb_image: RGB image (H, W, 3) numpy array
                - depth_image: depth image (H, W) numpy array, uint16
        """
        if not self.enable_depth:
            rgb_image = self.capture_rgb_frame()
            return rgb_image, None

        try:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.aligner.process(frames)

            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if depth_frame and color_frame:
                self.depth_image_np = np.asanyarray(depth_frame.get_data()).astype(np.uint16)
                self.color_image_np = np.asanyarray(color_frame.get_data())[:, :, :3]

                return self.color_image_np, self.depth_image_np

            return None, None

        except Exception as e:
            print(f"Error capturing frames: {e}")
            return None, None

    def capture_rgb_frame(self):
        """
        Capture an RGB frame only.

        Returns:
            numpy.ndarray: RGB image (H, W, 3)
        """
        try:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            
            if color_frame:
                self.color_image_np = np.asanyarray(color_frame.get_data())[:, :, :3]
                return self.color_image_np
            
            return None
            
        except Exception as e:
            print(f"Error capturing RGB frame: {e}")
            return None

    def capture_depth_frame(self):
        """
        Capture a depth frame only.

        Returns:
            numpy.ndarray: depth image (H, W), uint16
        """
        if not self.enable_depth:
            return None

        try:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.aligner.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            
            if depth_frame:
                self.depth_image_np = np.asanyarray(depth_frame.get_data()).astype(np.uint16)
                return self.depth_image_np
            
            return None
            
        except Exception as e:
            print(f"Error capturing depth frame: {e}")
            return None

    def close(self):
        """Close the camera and release resources."""
        try:
            self.pipeline.stop()
            cv2.destroyAllWindows()
        except Exception as e:
            print(f"Error closing camera: {e}")


if __name__ == "__main__":
    camera = RealSenseImage()
    
    try:
        while True:
            rgb, depth = camera.capture_rgb_depth_frames()
            
            if rgb is not None and depth is not None:
                cv2.imshow('RGB', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))

                # Normalize depth for display.
                depth_colormap = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth, alpha=0.03),
                    cv2.COLORMAP_JET
                )
                cv2.imshow('Depth', depth_colormap)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    finally:
        camera.close()

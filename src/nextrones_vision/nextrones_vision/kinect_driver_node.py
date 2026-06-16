#!/usr/bin/env python3
"""
Kinect v1 (Xbox 360 / Kinect for Windows) driver node using libfreenect.
Publishes RGB and depth images to standard /camera/* topics expected by
the rest of the Nextrones pipeline.

Requires: python3-freenect (sudo apt install libfreenect-dev freenect)
"""
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge
import numpy as np
import cv2


class KinectDriverNode(Node):

    def __init__(self):
        super().__init__('kinect_driver_node')

        self.rgb_pub = self.create_publisher(Image, '/camera/rgb/image_raw', 10)
        # Compressed RGB for streaming over WiFi to off-board YOLO (~40KB vs ~900KB
        # raw — raw never streams reliably over WiFi). Local nodes use the raw topic.
        self.rgb_comp_pub = self.create_publisher(
            CompressedImage, '/camera/rgb/image_raw/compressed', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.rgb_info_pub = self.create_publisher(CameraInfo, '/camera/rgb/camera_info', 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, '/camera/depth/camera_info', 10)

        self.bridge = CvBridge()

        # Standard Kinect v1 factory intrinsics (close enough for navigation)
        self.rgb_K = [525.0, 0.0, 319.5, 0.0, 525.0, 239.5, 0.0, 0.0, 1.0]
        self.depth_K = [585.05, 0.0, 315.83, 0.0, 585.05, 242.46, 0.0, 0.0, 1.0]

        try:
            import freenect
            self.freenect = freenect
            ctx = freenect.init()
            count = freenect.num_devices(ctx)
            # Release the probe context immediately — if we keep it open it holds
            # the USB device and the sync_get_* API below hangs forever (no frames).
            freenect.shutdown(ctx)
            if count == 0:
                self.get_logger().error(
                    'No Kinect device found! Check USB 3.0 connection and power.')
            else:
                self.get_logger().info(f'Kinect driver started — {count} device(s) found')
        except ImportError:
            self.get_logger().fatal(
                'freenect Python module not found. '
                'Run: sudo apt install libfreenect-dev python3-freenect')
            raise

        self._frame_count = 0
        self._running = True
        # Capture in a dedicated thread, NOT a ROS timer. Under the full launch
        # (YOLO + RTAB-Map + Nav2 on a 4-core Pi) the executor is starved and the
        # blocking sync_get_* never gets serviced, so zero frames are produced.
        # A dedicated thread keeps the USB stream alive independent of ROS load.
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()

    def _make_info(self, K, frame_id, stamp):
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = frame_id
        info.height = 480
        info.width = 640
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = K
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [K[0], 0.0, K[2], 0.0,  0.0, K[4], K[5], 0.0,  0.0, 0.0, 1.0, 0.0]
        return info

    @staticmethod
    def _raw_to_mm(raw):
        """Convert Kinect 11-bit raw disparity to millimetres."""
        mm = np.zeros(raw.shape, dtype=np.uint16)
        valid = (raw > 0) & (raw < 2047)
        # Kinect linearisation formula (Hendyanto / OpenKinect)
        mm[valid] = (
            1000.0 / (raw[valid].astype(np.float32) * -0.0030711016 + 3.3309495161)
        ).astype(np.uint16)
        return mm

    def _capture_loop(self):
        """Continuously grab frames in a dedicated thread (see __init__)."""
        while self._running and rclpy.ok():
            try:
                rgb = self.freenect.sync_get_video(0)
                raw_depth = self.freenect.sync_get_depth(0)
                if rgb is None or raw_depth is None:
                    self.get_logger().warn(
                        'Kinect returned no frame (device may be wedged — replug USB)',
                        throttle_duration_sec=3.0)
                    import time
                    time.sleep(0.1)
                    continue
                rgb, _ = rgb
                raw_depth, _ = raw_depth
            except Exception as exc:
                self.get_logger().warn(
                    f'Kinect read error: {exc}', throttle_duration_sec=3.0)
                import time
                time.sleep(0.1)
                continue

            self._publish(rgb, raw_depth)

            self._frame_count += 1
            if self._frame_count == 1 or self._frame_count % 30 == 0:
                self.get_logger().info(f'Captured & published {self._frame_count} frames')

    def _publish(self, rgb, raw_depth):

        stamp = self.get_clock().now().to_msg()

        rgb_msg = self.bridge.cv2_to_imgmsg(rgb, encoding='rgb8')
        rgb_msg.header.stamp = stamp
        rgb_msg.header.frame_id = 'camera_rgb_optical_frame'
        self.rgb_pub.publish(rgb_msg)
        self.rgb_info_pub.publish(
            self._make_info(self.rgb_K, 'camera_rgb_optical_frame', stamp))

        # JPEG-compressed RGB for off-board YOLO over WiFi. Convert RGB->BGR so the
        # decoded image matches the raw bgr8 path (consistent colours for YOLO).
        comp = self.bridge.cv2_to_compressed_imgmsg(
            cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), dst_format='jpg')
        comp.header = rgb_msg.header
        self.rgb_comp_pub.publish(comp)

        depth_mm = self._raw_to_mm(raw_depth)
        depth_msg = self.bridge.cv2_to_imgmsg(depth_mm, encoding='16UC1')
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = 'camera_depth_optical_frame'
        self.depth_pub.publish(depth_msg)
        self.depth_info_pub.publish(
            self._make_info(self.depth_K, 'camera_depth_optical_frame', stamp))


def main(args=None):
    rclpy.init(args=args)
    node = KinectDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the capture thread, then release the Kinect cleanly — without
        # sync_stop() the device is left wedged and the NEXT process sees
        # "1 device" but sync_get_* blocks forever.
        node._running = False
        try:
            node._capture_thread.join(timeout=2.0)
        except Exception:
            pass
        try:
            node.freenect.sync_stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

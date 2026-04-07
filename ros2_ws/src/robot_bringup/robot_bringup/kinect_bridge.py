#!/usr/bin/env python3
"""
Kinect v1 ROS 2 Bridge for Jazzy (headless, no libfreenect Python bindings needed).
Uses ctypes to call libfreenect directly from Python.

Publishes:
  /camera/color/image_raw       (sensor_msgs/Image, RGB8)
  /camera/depth/image_raw       (sensor_msgs/Image, 16UC1 in mm)
  /camera/color/camera_info     (sensor_msgs/CameraInfo)
  /camera/depth/camera_info     (sensor_msgs/CameraInfo)

Run: ros2 run robot_bringup kinect_bridge  (or directly with python3)
"""

import ctypes
import ctypes.util
import numpy as np
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
import builtin_interfaces.msg

# ───────────── libfreenect ctypes bindings ────────────────────────────────────
lib_path = ctypes.util.find_library("freenect")
if lib_path is None:
    lib_path = ctypes.util.find_library("freenect-sync")
    if lib_path is None:
        raise RuntimeError("libfreenect not found. Install: sudo apt install libfreenect-dev")

_fn = ctypes.CDLL(lib_path)

# We use the synchronous (blocking) freenect API for simplicity
lib_sync_path = ctypes.util.find_library("freenect_sync")
if lib_sync_path:
    _fns = ctypes.CDLL(lib_sync_path)
else:
    _fns = None  # will fall back to async API

FREENECT_VIDEO_RGB      = 0
FREENECT_DEPTH_11BIT    = 0
FREENECT_DEPTH_REGISTERED = 2

# Kinect v1 intrinsics (approximate, standard calibration)
# These are for V1 (RGBcam 640x480 @30fps, depth 640x480)
# These are for V1 (Downsampled to 320x240)
RGB_FX   = 262.5
RGB_FY   = 262.5
RGB_CX   = 159.75
RGB_CY   = 119.75
DEPTH_FX = 262.5
DEPTH_FY = 262.5
DEPTH_CX = 159.75
DEPTH_CY = 119.75
# ─────────────────────────────────────────────────────────────────────────────
DEPTH_OFFSET_MM = 0  # Standard offset (can be tuned via ROS parameters)


class KinectBridgeNode(Node):
    def __init__(self):
        super().__init__('kinect_bridge')

        qos = QoSProfile(depth=10)
        self.rgb_pub   = self.create_publisher(Image,      '/camera/color/image_raw',     qos)
        self.depth_pub = self.create_publisher(Image,      '/camera/depth/image_raw',      qos)
        self.rgb_info  = self.create_publisher(CameraInfo, '/camera/color/camera_info',   qos)
        self.depth_info= self.create_publisher(CameraInfo, '/camera/depth/camera_info',   qos)

        self._lock     = threading.Lock()
        self._rgb_buf  = None
        self._dep_buf  = None
        self._running  = True
        self._ts       = 0

        self.declare_parameter('depth_offset_mm', DEPTH_OFFSET_MM)

        # Try sync API first (simpler)
        if _fns:
            self._use_sync = True
            self.get_logger().info('Using freenect sync API')
        else:
            self._use_sync = False
            self._init_async_freenect()

        # Timer to publish frames at ~15 Hz
        self._grab_thread = threading.Thread(target=self._grab_loop, daemon=True)
        self._grab_thread.start()
        self.create_timer(1.0/15.0, self._publish_latest)

        self.get_logger().info('✅ Kinect bridge started — publishing RGB+depth to /camera/...')

    # ── Sync API (freenect_sync) ───────────────────────────────────────────────
    def _grab_loop(self):
        if self._use_sync:
            self._grab_loop_sync()
        else:
            self._grab_loop_async()

    def _grab_loop_sync(self):
        """Grab frames using freenect_sync_get_* (blocking but simple)."""
        # int freenect_sync_get_video(void **video, uint32_t *timestamp, int index, freenect_video_format fmt)
        _fns.freenect_sync_get_video.restype  = ctypes.c_int
        _fns.freenect_sync_get_video.argtypes = [
            ctypes.POINTER(ctypes.c_void_p), ctypes.POINTER(ctypes.c_uint32),
            ctypes.c_int, ctypes.c_int]
        _fns.freenect_sync_get_depth.restype  = ctypes.c_int
        _fns.freenect_sync_get_depth.argtypes = [
            ctypes.POINTER(ctypes.c_void_p), ctypes.POINTER(ctypes.c_uint32),
            ctypes.c_int, ctypes.c_int]

        rgb_ptr = ctypes.c_void_p()
        dep_ptr = ctypes.c_void_p()
        ts      = ctypes.c_uint32()

        while self._running:
            try:
                ret = _fns.freenect_sync_get_video(
                    ctypes.byref(rgb_ptr), ctypes.byref(ts), 0, FREENECT_VIDEO_RGB)
                if ret == 0 and rgb_ptr.value:
                    arr = np.frombuffer(
                        (ctypes.c_uint8 * (640*480*3)).from_address(rgb_ptr.value),
                        dtype=np.uint8).reshape((480, 640, 3))
                    # Downsample to 320x240 (1/4 data)
                    small_arr = arr[::2, ::2, :].copy()
                    with self._lock:
                        self._rgb_buf = small_arr
                        self._ts = ts.value

                ret = _fns.freenect_sync_get_depth(
                    ctypes.byref(dep_ptr), ctypes.byref(ts), 0, FREENECT_DEPTH_11BIT)
                if ret == 0 and dep_ptr.value:
                    raw = np.frombuffer(
                        (ctypes.c_uint16 * (640*480)).from_address(dep_ptr.value),
                        dtype=np.uint16).reshape((480, 640))
                    # Convert to mm
                    offset = self.get_parameter('depth_offset_mm').value
                    mm = self._raw_to_mm(raw, offset)
                    # Downsample to 320x240
                    small_dep = mm[::2, ::2].copy()
                    with self._lock:
                        self._dep_buf = small_dep
            except Exception as e:
                self.get_logger().warn(f'Frame grab error: {e}')
                time.sleep(0.1)

    def _raw_to_mm(self, raw, offset=0):
        """Convert Kinect 11-bit raw disparity to millimetres with calibration offset."""
        with np.errstate(divide='ignore', invalid='ignore'):
            # Standard conversion formula for Kinect V1 disparity
            mm = np.where(raw > 0, (1.0 / (raw * -0.0030711016 + 3.3309495161)) * 1000.0, 0)
            # Apply calibration offset (only where data is valid)
            mm = np.where(mm > 0, mm + offset, 0)
        return mm.astype(np.uint16)

    # ── Async API fallback (libfreenect) ──────────────────────────────────────
    def _init_async_freenect(self):
        self.get_logger().warn('freenect_sync not available — using async API')
        # Not implemented here; you can extend if needed

    def _grab_loop_async(self):
        self.get_logger().error('Async freenect fallback not implemented.')
        self._running = False

    # ── Publishing ─────────────────────────────────────────────────────────────
    def _publish_latest(self):
        with self._lock:
            rgb = self._rgb_buf
            dep = self._dep_buf
            ts  = self._ts

        stamp = self.get_clock().now().to_msg()
        frame_id = 'camera_link'

        if rgb is not None:
            self.rgb_pub.publish(self._to_image(rgb, 'rgb8', stamp, frame_id))
            self.rgb_info.publish(self._camera_info(stamp, frame_id, RGB_FX, RGB_FY, RGB_CX, RGB_CY))

        if dep is not None:
            self.depth_pub.publish(self._to_image(dep, '16UC1', stamp, frame_id))
            self.depth_info.publish(self._camera_info(stamp, frame_id, DEPTH_FX, DEPTH_FY, DEPTH_CX, DEPTH_CY))

    def _to_image(self, arr, encoding, stamp, frame_id):
        msg = Image()
        msg.header.stamp    = stamp
        msg.header.frame_id = frame_id
        msg.height   = arr.shape[0]
        msg.width    = arr.shape[1]
        msg.encoding = encoding
        msg.is_bigendian = False
        msg.step     = arr.strides[0]
        msg.data     = arr.tobytes()
        return msg

    def _camera_info(self, stamp, frame_id, fx, fy, cx, cy):
        msg = CameraInfo()
        msg.header.stamp    = stamp
        msg.header.frame_id = frame_id
        msg.width  = 320
        msg.height = 240
        msg.distortion_model = 'plumb_bob'
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [fx, 0.0, cx,  0.0, fy, cy,  0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0,  0.0, 1.0, 0.0,  0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0,  0.0, fy, cy, 0.0,  0.0, 0.0, 1.0, 0.0]
        return msg

    def destroy_node(self):
        self._running = False
        if _fns and self._use_sync:
            _fns.freenect_sync_stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KinectBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

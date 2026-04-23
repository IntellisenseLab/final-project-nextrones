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
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from std_msgs.msg import Header
import builtin_interfaces.msg
import cv2
from http.server import BaseHTTPRequestHandler, HTTPServer
import io

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
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        qos = QoSProfile(depth=10)
        qos_best_effort = QoSProfile(depth=1)
        qos_best_effort.reliability = ReliabilityPolicy.BEST_EFFORT
        
        self.rgb_pub   = self.create_publisher(Image,           '/camera/color/image_raw',     qos)
        self.rgb_comp  = self.create_publisher(CompressedImage, '/camera/color/image_raw/compressed', qos_best_effort)
        self.depth_pub = self.create_publisher(Image,           '/camera/depth/image_raw',      qos)
        self.rgb_info  = self.create_publisher(CameraInfo,      '/camera/color/camera_info',   qos)
        self.depth_info= self.create_publisher(CameraInfo,      '/camera/depth/camera_info',   qos)

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

        # Start MJPEG Web Server on port 5000
        self._latest_jpeg = None
        self._web_thread = threading.Thread(target=self._run_web_server, daemon=True)
        self._web_thread.start()

        # Start Grab Loop Thread
        self._grab_thread = threading.Thread(target=self._grab_loop, daemon=True)
        self._grab_thread.start()

        # Start timer for publishing (to keep it decoupled from grab rate)
        self.create_timer(1.0/30.0, self._publish_latest)

        self.get_logger().info('✅ Kinect bridge started — publishing to /camera/ and web http://0.0.0.0:5000')

    def _run_web_server(self):
        class MJPEGHandler(BaseHTTPRequestHandler):
            def do_GET(handler):
                if handler.path == '/':
                    handler.send_response(200)
                    handler.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
                    handler.end_headers()
                    try:
                        while True:
                            with self._lock:
                                jpeg = self._latest_jpeg
                            if jpeg is not None:
                                handler.wfile.write(b'--frame\r\n')
                                handler.send_header('Content-type', 'image/jpeg')
                                handler.send_header('Content-length', len(jpeg))
                                handler.end_headers()
                                handler.wfile.write(jpeg)
                                handler.wfile.write(b'\r\n')
                            time.sleep(0.05)
                    except Exception as e: 
                        self.get_logger().debug(f'Client disconnected: {e}')
                        return
                else:
                    handler.send_error(404)
            def log_message(self, format, *args): return # Silent

        server = HTTPServer(('0.0.0.0', 5000), MJPEGHandler)
        server.serve_forever()

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
                self.get_logger().debug('Grabbing RGB frame...')
                ret = _fns.freenect_sync_get_video(
                    ctypes.byref(rgb_ptr), ctypes.byref(ts), 0, FREENECT_VIDEO_RGB)
                if ret == 0 and rgb_ptr.value:
                    try:
                        arr = np.frombuffer(
                            (ctypes.c_uint8 * (640*480*3)).from_address(rgb_ptr.value),
                            dtype=np.uint8).reshape((480, 640, 3))
                        # Downsample to 320x240 (1/4 data)
                        small_arr = arr[::2, ::2, :].copy()
                        with self._lock:
                            self._rgb_buf = small_arr
                            self._ts = ts.value
                    except Exception as e:
                        self.get_logger().error(f'❌ Kinect Error: RGB buffer processing failed: {e}')

                ret = _fns.freenect_sync_get_depth(
                    ctypes.byref(dep_ptr), ctypes.byref(ts), 0, FREENECT_DEPTH_11BIT)
                if ret == 0 and dep_ptr.value:
                    try:
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
                        self.get_logger().error(f'❌ Kinect Error: Depth buffer processing failed: {e}')
            except Exception as e:
                self.get_logger().warn(f'⚠️ Kinect Grab Error: {e}. Retrying in 1s...')
                # Reset pointers if they were corrupted
                rgb_ptr = ctypes.c_void_p()
                dep_ptr = ctypes.c_void_p()
                time.sleep(1.0)

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
        # self.get_logger().info('Publish timer ticked') # Too verbose, leave off for now
        with self._lock:
            rgb = self._rgb_buf
            dep = self._dep_buf
            ts  = self._ts

        stamp = self.get_clock().now().to_msg()
        frame_id = 'camera_link'

        if rgb is not None:
            try:
                # Publish raw image (for local use)
                self.rgb_pub.publish(self._to_image(rgb, 'rgb8', stamp, 'camera_color_optical_frame'))
                
                # Publish compressed image (for WiFi/Laptop use)
                try:
                    comp_msg = CompressedImage()
                    comp_msg.header.stamp = stamp
                    comp_msg.header.frame_id = 'camera_color_optical_frame'
                    comp_msg.format = "jpeg"
                    # Encode RGB to JPEG (CV2 expects BGR for imencode)
                    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                    success, encoded_img = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                    if success:
                        comp_msg.data = encoded_img.tobytes()
                        self.rgb_comp.publish(comp_msg)
                        with self._lock:
                            self._latest_jpeg = comp_msg.data
                        
                        if not hasattr(self, '_frame_count'): self._frame_count = 0
                        self._frame_count += 1
                        if self._frame_count % 100 == 0:
                            self.get_logger().info('📡 Kinect Streaming: Heartbeat OK (100 frames processed)')
                    else:
                        self.get_logger().error('❌ Kinect Error: JPEG encoding failed!')
                except Exception as e:
                    self.get_logger().error(f'❌ Kinect Error: Compression failed: {e}')

                self.rgb_info.publish(self._camera_info(stamp, 'camera_color_optical_frame', RGB_FX, RGB_FY, RGB_CX, RGB_CY))
            except Exception as e:
                self.get_logger().error(f'❌ Kinect Error: RGB publication failed: {e}')

        if dep is not None:
            try:
                self.depth_pub.publish(self._to_image(dep, '16UC1', stamp, 'camera_depth_optical_frame'))
                self.depth_info.publish(self._camera_info(stamp, 'camera_depth_optical_frame', DEPTH_FX, DEPTH_FY, DEPTH_CX, DEPTH_CY))
            except Exception as e:
                self.get_logger().error(f'❌ Kinect Error: Depth publication failed: {e}')

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

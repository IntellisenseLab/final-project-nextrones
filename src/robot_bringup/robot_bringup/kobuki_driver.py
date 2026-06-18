#!/usr/bin/env python3
"""
Kobuki Python Driver for ROS 2 Jazzy
Subscribes to /cmd_vel and sends velocity commands directly over USB serial.
Bypasses kobuki_core (C++ stack incompatible with GCC 13 / Jazzy ECL deps).

Kobuki serial protocol (Little Endian):
  Header1: 0xAA
  Header2: 0x55
  Length: N  (bytes remaining in payload)
  [Payload]
  Checksum: XOR of (Length + all payload bytes)

Base Control (sub-payload type 0x01, 4 bytes):
  Speed  (int16): mm/s   (linear.x * 1000)
  Radius (int16): mm     (turning radius; 0x8000 = pure rotation)
"""

import struct
import threading
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
import serial
import serial.tools.list_ports


# Kobuki Physical Constants
TICKS_PER_REV = 2578.33
WHEEL_RADIUS = 0.035   # 70mm diameter
WHEEL_BASE = 0.230     # 230mm
TICK_TO_METER = (2.0 * math.pi * WHEEL_RADIUS) / TICKS_PER_REV


def find_kobuki_port():
    """Auto-detect Kobuki USB port."""
    for port in serial.tools.list_ports.comports():
        vid_pid = f"{port.vid:04x}:{port.pid:04x}" if port.vid else ""
        if "kobuki" in port.description.lower() or \
           "0403:6001" in vid_pid or "10c4:ea60" in vid_pid or \
           port.device in ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0"]:
            return port.device
    return "/dev/ttyUSB0"


def make_base_control_payload(speed_mms: int, radius_mm: int) -> bytes:
    """Build a Kobuki base control serial packet."""
    sub_payload = struct.pack('<BBhh', 0x01, 4, speed_mms, radius_mm)
    length = len(sub_payload)
    checksum = length
    for b in sub_payload:
        checksum ^= b
    packet = bytes([0xAA, 0x55, length]) + sub_payload + bytes([checksum])
    return packet


def twist_to_speed_radius(linear_x: float, angular_z: float):
    """Convert ROS Twist to Kobuki speed/radius."""
    speed_mms = int(linear_x * 1000)
    if abs(angular_z) < 1e-6:
        radius_mm = 0
    elif abs(linear_x) < 1e-6:
        speed_mms = int(angular_z * 165)
        radius_mm = 1
    else:
        radius_mm = int((linear_x / angular_z) * 1000)
        radius_mm = max(-32767, min(32767, radius_mm))
    return speed_mms, radius_mm


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = Quaternion()
    q.x = sj*sc - cj*ss
    q.y = cj*sc + sj*ss
    q.z = cj*cs - sj*sc
    q.w = cj*cc + sj*sk
    return q


class KobukiDriverNode(Node):
    def __init__(self):
        super().__init__('kobuki_driver')

        self.declare_parameter('port', '')
        self.declare_parameter('baud', 115200)

        port_param = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        if not port_param:
            port_param = find_kobuki_port()

        self.get_logger().info(f'Opening Kobuki on {port_param} @ {baud} baud')

        try:
            self.serial = serial.Serial(port_param, baud, timeout=0.1)
            self.connected = True
            self.get_logger().info('✅ Kobuki serial port opened!')
            # Beep once to confirm connection physically
            beep_packet = bytes([0xAA, 0x55, 0x03, 0x03, 0x03, 0x01, 0x00, 0x01]) # Simplified beep
            # Actually, let's use a proper sub-payload
            # Type 3 (Sound), Length 3, Note 0x01, Duration 0x01
            # Checksum = 3 ^ 3 ^ 3 ^ 1 ^ 0 = 4? No, let's just use the correct XOR sum.
            self.serial.write(bytes([0xAA, 0x55, 0x04, 0x03, 0x02, 0x01, 0x00, 0x04])) 
        except serial.SerialException as e:
            self.get_logger().error(f'❌ Failed to open serial port: {e}')
            self.serial = None
            self.connected = False

        # Odometry State
        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.last_l_ticks = None
        self.last_r_ticks = None

        # ROS Publishers/Subscribers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        # Background Reader Thread
        self._stop_event = threading.Event()
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        if self.connected:
            self.read_thread.start()

        self.get_logger().info('Kobuki driver ready with Odometry support.')

    def cmd_vel_cb(self, msg: Twist):
        speed_mms, radius_mm = twist_to_speed_radius(msg.linear.x, msg.angular.z)
        packet = make_base_control_payload(speed_mms, radius_mm)
        self.get_logger().info(f'Received cmd_vel: lin={msg.linear.x:.2f}, ang={msg.angular.z:.2f} -> speed={speed_mms}, rad={radius_mm}')
        if self.connected and self.serial:
            try:
                self.serial.write(packet)
                self.get_logger().info('Sent packet to serial.')
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write error: {e}')

    def _read_loop(self):
        """Read and parse feedback packets from Kobuki."""
        while not self._stop_event.is_set() and self.connected:
            try:
                # Find Header 0xAA 0x55
                b = self.serial.read(1)
                if not b or b[0] != 0xAA: continue
                b = self.serial.read(1)
                if not b or b[0] != 0x55: continue

                # Read Length
                length_b = self.serial.read(1)
                if not length_b: continue
                length = length_b[0]

                # Read Payload + Checksum
                payload = self.serial.read(length)
                checksum_b = self.serial.read(1)
                if len(payload) < length or not checksum_b: continue
                
                # Checksum verify (XOR of all payload bytes + length)
                cal_checksum = length
                for p in payload: cal_checksum ^= p
                if cal_checksum != checksum_b[0]:
                    self.get_logger().warn('Checksum mismatch!')
                    continue

                # Parse sub-payloads
                idx = 0
                while idx + 1 < len(payload):
                    sub_id = payload[idx]
                    sub_len = payload[idx+1]
                    
                    # Guard against malformed sub-payload lengths
                    if idx + 2 + sub_len > len(payload):
                        break
                        
                    sub_data = payload[idx+2 : idx+2+sub_len]
                    
                    if sub_id == 0x01: # Basic Sensor Data
                        self._handle_sensor_data(sub_data)
                    
                    idx += 2 + sub_len


            except Exception as e:
                self.get_logger().error(f'Read loop error: {e}')

    def _handle_sensor_data(self, data):
        """Extract encoder ticks and update odometry."""
        # ID 0x01 Basic Sensor Data is 15 bytes total:
        # TS(2), Bumper(1), WheelDrop(1), Cliff(1), L_Enc(2), R_Enc(2), 
        # L_PWM(1), R_PWM(1), Button(1), Charger(1), Battery(1)
        if len(data) < 15:
            return

        try:
            # Unpack the first 9 bytes to get encoders
            # H=TS, B=Bumper, B=WheelDrop, B=Cliff, H=L_Enc, H=R_Enc
            # 2 + 1 + 1 + 1 + 2 + 2 = 9 bytes
            _, _, _, _, l_ticks, r_ticks = struct.unpack('<HBBBHH', data[:9])
        except struct.error as e:
            self.get_logger().error(f'Unpack error: {e} | data_len: {len(data)}')
            return

        if self.last_l_ticks is None:
            self.last_l_ticks = l_ticks
            self.last_r_ticks = r_ticks
            return

        # Handle 16-bit wrap-around
        def tick_diff(curr, last):
            diff = curr - last
            if diff > 32768: diff -= 65536
            if diff < -32768: diff += 65536
            return diff

        dl = tick_diff(l_ticks, self.last_l_ticks) * TICK_TO_METER
        dr = tick_diff(r_ticks, self.last_r_ticks) * TICK_TO_METER
        
        self.last_l_ticks = l_ticks
        self.last_r_ticks = r_ticks

        # Differential Kinematics
        d_dist = (dr + dl) / 2.0
        d_th = (dr - dl) / WHEEL_BASE

        # Update pose
        self.x += d_dist * math.cos(self.th + (d_th / 2.0))
        self.y += d_dist * math.sin(self.th + (d_th / 2.0))
        self.th += d_th

        self._publish_odom()

    def _publish_odom(self):
        now = self.get_clock().now().to_msg()
        q = quaternion_from_euler(0, 0, self.th)

        # 1. Publish Odometry Message
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        self.odom_pub.publish(odom)

        # 2. Publish TF Transform
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        self._stop_event.set()
        if self.serial and self.serial.is_open:
            stop = make_base_control_payload(0, 0)
            try: self.serial.write(stop)
            except: pass
            self.serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KobukiDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

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
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import serial.tools.list_ports


def find_kobuki_port():
    """Auto-detect Kobuki USB port."""
    for port in serial.tools.list_ports.comports():
        # Kobuki shows up as FTDI or Silicon Labs chip
        vid_pid = f"{port.vid:04x}:{port.pid:04x}" if port.vid else ""
        if "kobuki" in port.description.lower() or \
           "0403:6001" in vid_pid or "10c4:ea60" in vid_pid or \
           port.device in ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0"]:
            return port.device
    return "/dev/ttyUSB0"  # fallback


def make_base_control_payload(speed_mms: int, radius_mm: int) -> bytes:
    """Build a Kobuki base control serial packet."""
    # Sub-payload: type=0x01, length=4
    sub_payload = struct.pack('<BBhh', 0x01, 4, speed_mms, radius_mm)
    length = len(sub_payload)
    checksum = length
    for b in sub_payload:
        checksum ^= b
    packet = bytes([0xAA, 0x55, length]) + sub_payload + bytes([checksum])
    return packet


def twist_to_speed_radius(linear_x: float, angular_z: float):
    """Convert ROS Twist to Kobuki speed/radius."""
    speed_mms = int(linear_x * 1000)   # m/s → mm/s
    if abs(angular_z) < 1e-6:
        radius_mm = 0  # straight
    elif abs(linear_x) < 1e-6:
        # Pure rotation: Kobuki uses 1 mm radius to signal spin
        speed_mms = int(angular_z * 165)  # empirical: wheel_base/2 = 165mm
        radius_mm = 1
    else:
        # radius = linear / angular
        radius_mm = int((linear_x / angular_z) * 1000)
        radius_mm = max(-32767, min(32767, radius_mm))
    return speed_mms, radius_mm


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
        except serial.SerialException as e:
            self.get_logger().error(f'❌ Failed to open serial port: {e}')
            self.get_logger().warn('Running in DRY-RUN mode (no hardware)')
            self.serial = None
            self.connected = False

        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.get_logger().info('Kobuki driver ready. Listening on /cmd_vel ...')

    def cmd_vel_cb(self, msg: Twist):
        speed_mms, radius_mm = twist_to_speed_radius(msg.linear.x, msg.angular.z)
        packet = make_base_control_payload(speed_mms, radius_mm)
        # Throttled logging
        if not hasattr(self, '_cmd_count'): self._cmd_count = 0
        self._cmd_count += 1
        if self._cmd_count % 20 == 0:
            self.get_logger().info(
                f'CMD_VEL ({self._cmd_count}) → speed={speed_mms}mm/s radius={radius_mm}mm'
            )
        if self.connected and self.serial and self.serial.is_open:
            try:
                self.serial.write(packet)
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write error: {e}')

    def _read_loop(self):
        while rclpy.ok() and self.connected:
            try:
                if not self.serial or not self.serial.is_open:
                    time.sleep(0.1)
                    continue
                # Read header
                header = self.serial.read(2)
            except Exception:
                pass

    def destroy_node(self):
        if self.serial and self.serial.is_open:
            # Send stop command before shutting down
            stop = make_base_control_payload(0, 0)
            try:
                self.serial.write(stop)
            except Exception:
                pass
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

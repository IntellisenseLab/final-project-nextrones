import os
import subprocess
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class HardwareMonitorNode(Node):
    """
    Monitors hardware health on the Raspberry Pi and publishes a status report.

    Publishes:
      /nextrones/diagnostics  (diagnostic_msgs/DiagnosticArray) — machine-readable
      /nextrones/status_text  (std_msgs/String)                 — human-readable summary

    Also writes errors to a log file so diagnose.sh can surface them.
    """

    KOBUKI_USB_ID = '0403:6001'
    KINECT_USB_IDS = ['045e:02ae', '045e:02bf', '045e:02c2']

    REQUIRED_TOPICS = [
        '/odom',
        '/camera/rgb/image_raw',
        '/camera/depth/image_raw',
        '/scan_filtered',
    ]

    def __init__(self):
        super().__init__('hardware_monitor_node')
        self.declare_parameter('check_interval', 5.0)
        interval = self.get_parameter('check_interval').get_parameter_value().double_value

        self.diag_pub = self.create_publisher(DiagnosticArray, '/nextrones/diagnostics', 10)
        self.text_pub = self.create_publisher(String, '/nextrones/status_text', 10)

        self._topic_last_seen = {t: None for t in self.REQUIRED_TOPICS}
        self._topic_subs = []
        for topic in self.REQUIRED_TOPICS:
            from sensor_msgs.msg import Image, LaserScan
            from nav_msgs.msg import Odometry
            type_map = {
                '/odom': Odometry,
                '/camera/rgb/image_raw': Image,
                '/camera/depth/image_raw': Image,
                '/scan_filtered': LaserScan,
            }
            msg_type = type_map.get(topic, String)
            self._topic_subs.append(
                self.create_subscription(msg_type, topic,
                    lambda msg, t=topic: self._topic_seen(t), 1)
            )

        self.create_timer(interval, self._check_all)
        self.get_logger().info('Hardware Monitor started — publishing to /nextrones/diagnostics')

    def _topic_seen(self, topic):
        self._topic_last_seen[topic] = time.time()

    def _lsusb(self):
        try:
            result = subprocess.run(['lsusb'], capture_output=True, text=True, timeout=3)
            return result.stdout
        except Exception:
            return ''

    def _check_kobuki(self):
        lsusb_out = self._lsusb()
        if self.KOBUKI_USB_ID in lsusb_out:
            return True, 'USB connected'
        if os.path.exists('/dev/kobuki'):
            return True, '/dev/kobuki present'
        return False, 'Not found — check USB cable and udev rules (57-kobuki.rules)'

    def _check_kinect(self):
        lsusb_out = self._lsusb()
        for uid in self.KINECT_USB_IDS:
            if uid in lsusb_out:
                return True, f'USB connected ({uid})'
        return False, 'Not found — check USB cable, try USB 3.0 port'

    def _check_ram(self):
        try:
            import psutil
            mem = psutil.virtual_memory()
            free_mb = mem.available // (1024 * 1024)
            pct = mem.percent
            ok = free_mb > 300
            msg = f'{free_mb} MB free ({pct:.0f}% used)'
            if not ok:
                msg += ' — WARNING: low memory, consider closing other apps'
            return ok, msg
        except ImportError:
            return True, 'psutil not installed — skipping RAM check'

    def _check_cpu_temp(self):
        try:
            with open('/sys/class/thermal/thermal_zone0/temp') as f:
                temp_mc = int(f.read().strip())
            temp_c = temp_mc / 1000.0
            ok = temp_c < 80.0
            msg = f'{temp_c:.1f}°C'
            if not ok:
                msg += ' — WARNING: throttling likely, add cooling'
            return ok, msg
        except Exception:
            return True, 'N/A (not on Pi or no thermal zone)'

    def _check_topics(self):
        now = time.time()
        stale = []
        for topic, last in self._topic_last_seen.items():
            if last is None or (now - last) > 5.0:
                stale.append(topic)
        if stale:
            return False, f'Not publishing: {", ".join(stale)}'
        return True, 'All required topics publishing'

    def _check_all(self):
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()

        checks = [
            ('Kobuki USB',      self._check_kobuki),
            ('Kinect USB',      self._check_kinect),
            ('RAM',             self._check_ram),
            ('CPU Temperature', self._check_cpu_temp),
            ('Required Topics', self._check_topics),
        ]

        all_ok = True
        summary_lines = []

        for name, fn in checks:
            try:
                ok, msg = fn()
            except Exception as e:
                ok, msg = False, f'Check failed: {e}'

            status = DiagnosticStatus()
            status.name = f'Nextrones/{name}'
            status.message = msg
            status.level = DiagnosticStatus.OK if ok else DiagnosticStatus.ERROR
            status.values.append(KeyValue(key='detail', value=msg))
            array.status.append(status)

            icon = '✓' if ok else '✗'
            summary_lines.append(f'{icon} {name}: {msg}')
            if not ok:
                all_ok = False
                self.get_logger().error(f'[{name}] {msg}')

        self.diag_pub.publish(array)

        summary = ('ALL OK' if all_ok else 'ISSUES DETECTED') + '\n' + '\n'.join(summary_lines)
        self.text_pub.publish(String(data=summary))

        if all_ok:
            self.get_logger().info('Hardware OK')
        else:
            self.get_logger().warn('Hardware issues detected — run scripts/diagnose.sh for details')


def main(args=None):
    rclpy.init(args=args)
    node = HardwareMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

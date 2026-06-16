#!/bin/bash
# Run on Raspberry Pi (directly or via SSH) to capture full system diagnostics.
# Claude Code can parse this output to identify hardware and software issues.
#
# Usage (on Pi):       bash ~/ros_final/scripts/diagnose.sh
# Usage (from PC):     ssh pi@<IP> 'bash ~/ros_final/scripts/diagnose.sh'
# Usage (via claude):  bash scripts/ssh_debug.sh pi@<IP>

source /opt/ros/jazzy/setup.bash 2>/dev/null
source ~/ros_final/install/setup.bash 2>/dev/null

SEP="────────────────────────────────────"

echo "╔══════════════════════════════════════╗"
echo "║    NEXTRONES DIAGNOSTICS REPORT      ║"
echo "╚══════════════════════════════════════╝"
echo "Timestamp: $(date)"
echo "Hostname:  $(hostname)"
echo ""

# ── HARDWARE ──────────────────────────────
echo "$SEP"
echo "HARDWARE"
echo "$SEP"

echo "▸ Kobuki USB:"
if ls /dev/kobuki 2>/dev/null; then
    echo "  STATUS: CONNECTED at /dev/kobuki"
elif lsusb | grep -qi "0403:6001"; then
    echo "  STATUS: CONNECTED (udev symlink not created yet — check udev rules)"
else
    echo "  STATUS: NOT FOUND — check USB cable and kobuki udev rule"
fi

echo ""
echo "▸ Kinect USB:"
if lsusb | grep -qiE "045e:02ae|045e:02bf|045e:02c2"; then
    echo "  STATUS: CONNECTED"
    lsusb | grep -iE "045e:02"
else
    echo "  STATUS: NOT FOUND — check USB connection (needs USB 3.0 port)"
fi

echo ""
echo "▸ All USB devices:"
lsusb

# ── SYSTEM RESOURCES ──────────────────────
echo ""
echo "$SEP"
echo "SYSTEM RESOURCES"
echo "$SEP"
echo "▸ RAM:"
free -h
echo ""
echo "▸ CPU Temperature:"
if [ -f /sys/class/thermal/thermal_zone0/temp ]; then
    TEMP=$(cat /sys/class/thermal/thermal_zone0/temp)
    echo "  $(echo "scale=1; $TEMP/1000" | bc)°C  $([ $TEMP -gt 80000 ] && echo '⚠ WARNING: THROTTLING' || echo 'OK')"
else
    echo "  N/A"
fi
echo ""
echo "▸ Disk:"
df -h / | tail -1
echo ""
echo "▸ Swap:"
swapon --show || echo "  No swap active"
echo ""
echo "▸ CPU load:"
uptime

# ── ROS 2 STATUS ──────────────────────────
echo ""
echo "$SEP"
echo "ROS 2 STATUS"
echo "$SEP"
echo "▸ Running nodes:"
ros2 node list 2>/dev/null || echo "  No nodes running (is the launch file started?)"

echo ""
echo "▸ Active topics:"
ros2 topic list 2>/dev/null | head -40 || echo "  No topics"

# ── TOPIC HEALTH ──────────────────────────
echo ""
echo "$SEP"
echo "TOPIC HEALTH (1-second sample)"
echo "$SEP"
TOPICS=(
    "/odom"
    "/scan_filtered"
    "/camera/rgb/image_raw"
    "/camera/depth/image_raw"
    "/nextrones/detections"
    "/nextrones/localized_objects"
    "/nextrones/semantic_markers"
    "/cmd_vel"
    "/tf"
)
for topic in "${TOPICS[@]}"; do
    printf "  %-40s " "$topic:"
    RATE=$(timeout 2 ros2 topic hz "$topic" 2>/dev/null | grep "average rate" | awk '{print $3}' | head -1)
    if [ -n "$RATE" ]; then
        echo "✓ ${RATE} Hz"
    else
        echo "✗ NOT PUBLISHING"
    fi
done

# ── TF TREE ───────────────────────────────
echo ""
echo "$SEP"
echo "TF FRAMES"
echo "$SEP"
ros2 run tf2_tools view_frames 2>/dev/null &
TF_PID=$!
sleep 2
kill $TF_PID 2>/dev/null
ros2 topic echo /tf --once 2>/dev/null | grep "frame_id\|child_frame" | head -20 || echo "  TF not available"

# ── RECENT LOGS ───────────────────────────
echo ""
echo "$SEP"
echo "RECENT ERRORS (last 30 lines)"
echo "$SEP"
LOG_DIR="${ROS_LOG_DIR:-$HOME/.ros/log}"
LATEST=$(ls -t "$LOG_DIR"/ 2>/dev/null | head -1)
if [ -n "$LATEST" ]; then
    grep -h -iE "error|fatal|exception|crash" "$LOG_DIR/$LATEST/"*.log 2>/dev/null | tail -30 || echo "  No errors found in logs"
else
    echo "  No log directory found at $LOG_DIR"
fi

# ── NEXTRONES PACKAGE CHECK ───────────────
echo ""
echo "$SEP"
echo "INSTALLED PACKAGES"
echo "$SEP"
for pkg in nextrones_vision nextrones_localization nextrones_mapping nextrones_navigation nextrones_diagnostics; do
    if ros2 pkg list 2>/dev/null | grep -q "$pkg"; then
        echo "  ✓ $pkg"
    else
        echo "  ✗ $pkg  (not built — run build_pi.sh)"
    fi
done

echo ""
echo "▸ Python packages:"
python3 -c "import ultralytics; print('  ✓ ultralytics', ultralytics.__version__)" 2>/dev/null || echo "  ✗ ultralytics NOT installed"
python3 -c "import numpy; print('  ✓ numpy', numpy.__version__)" 2>/dev/null || echo "  ✗ numpy NOT installed"
python3 -c "import cv2; print('  ✓ opencv', cv2.__version__)" 2>/dev/null || echo "  ✗ opencv NOT installed"

echo ""
echo "▸ YOLO model:"
if [ -d ~/ros_final/yolov8n_ncnn_model ]; then
    echo "  ✓ NCNN model at ~/ros_final/yolov8n_ncnn_model  (fast ARM inference)"
elif [ -f ~/ros_final/yolov8n.pt ]; then
    echo "  ✓ PyTorch model at ~/ros_final/yolov8n.pt  (slow — convert to NCNN for 3x speedup)"
else
    echo "  ✗ No YOLO model found — copy yolov8n.pt or yolov8n_ncnn_model/ to ~/ros_final/"
fi

echo ""
echo "╔══════════════════════════════════════╗"
echo "║          END OF DIAGNOSTICS          ║"
echo "╚══════════════════════════════════════╝"

#!/usr/bin/env python3
"""
Convert yolov8n.pt to NCNN format for ~3x faster inference on Raspberry Pi ARM CPU.
Run this ONCE on your PC (not on the Pi — export needs more RAM).

Usage:
    python3 scripts/convert_yolo_ncnn.py

Output: yolov8n_ncnn_model/ directory in ros_final/
Then sync_to_pi.sh will copy it to the Pi automatically.
"""
import os
import sys

try:
    from ultralytics import YOLO
except ImportError:
    print("Install ultralytics first: pip3 install ultralytics")
    sys.exit(1)

workspace = os.path.join(os.path.dirname(__file__), '..')
pt_path = os.path.join(workspace, 'yolov8n.pt')
out_path = os.path.join(workspace, 'yolov8n_ncnn_model')

if not os.path.exists(pt_path):
    print(f"Downloading yolov8n.pt to {pt_path}...")
    model = YOLO('yolov8n.pt')
    import shutil
    shutil.copy('yolov8n.pt', pt_path)
else:
    model = YOLO(pt_path)

print("Exporting to NCNN format...")
model.export(format='ncnn', imgsz=640)

# Ultralytics exports to yolov8n_ncnn_model/ in cwd
import shutil
if os.path.exists('yolov8n_ncnn_model'):
    if os.path.exists(out_path):
        shutil.rmtree(out_path)
    shutil.move('yolov8n_ncnn_model', out_path)

print(f"\nNCNN model saved to: {out_path}")
print("Now run:  bash scripts/sync_to_pi.sh pi@<IP>")
print("The Pi launch file will automatically use the NCNN model.")

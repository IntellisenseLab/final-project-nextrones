#!/bin/bash
# Sync workspace from PC to Raspberry Pi over SSH.
# Usage: bash sync_to_pi.sh pi@192.168.x.x
#
# What it does:
#   - rsyncs src/ scripts/ and YOLO model to the Pi
#   - Excludes build/ install/ log/ to save time
#   - Does NOT build — run build_pi.sh on the Pi after sync

set -e

if [ -z "$1" ]; then
    echo "Usage: $0 <user@pi-ip>"
    echo "Example: $0 pi@192.168.1.50"
    exit 1
fi

PI="$1"
WORKSPACE="$(cd "$(dirname "$0")/.." && pwd)"

echo "Syncing $WORKSPACE → $PI:~/ros_final ..."

# Ensure target directory exists
ssh "$PI" "mkdir -p ~/ros_final"

rsync -avz --progress \
    --exclude='build/' \
    --exclude='install/' \
    --exclude='log/' \
    --exclude='__pycache__/' \
    --exclude='*.pyc' \
    --exclude='.git/' \
    "$WORKSPACE/src" \
    "$WORKSPACE/scripts" \
    "$PI:~/ros_final/"

# Sync YOLO model separately (large file, only if changed)
if [ -f "$WORKSPACE/yolov8n.pt" ]; then
    echo "Syncing yolov8n.pt..."
    rsync -avz --progress "$WORKSPACE/yolov8n.pt" "$PI:~/ros_final/"
fi
if [ -d "$WORKSPACE/yolov8n_ncnn_model" ]; then
    echo "Syncing NCNN model..."
    rsync -avz --progress "$WORKSPACE/yolov8n_ncnn_model" "$PI:~/ros_final/"
fi

echo ""
echo "Sync complete. Now run on the Pi:"
echo "  ssh $PI"
echo "  bash ~/ros_final/scripts/build_pi.sh"

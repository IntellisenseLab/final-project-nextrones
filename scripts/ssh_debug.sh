#!/bin/bash
# Run FROM YOUR PC to remotely diagnose the Raspberry Pi over SSH.
# Claude Code uses this script to debug Pi deployment issues.
#
# Usage: bash scripts/ssh_debug.sh pi@192.168.x.x
# Example: bash scripts/ssh_debug.sh pi@192.168.1.50

set -e

if [ -z "$1" ]; then
    echo "Usage: $0 <user@pi-ip>"
    echo "Example: $0 pi@192.168.1.50"
    exit 1
fi

PI="$1"
echo "=== Connecting to $PI for diagnostics ==="
ssh -o ConnectTimeout=10 "$PI" 'bash ~/ros_final/scripts/diagnose.sh'

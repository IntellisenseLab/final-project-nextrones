#!/bin/bash
# A utility shell script to launch the openni2_camera node and open rqt_image_view
# to verify the RGB and Depth streams.

# Remove the PlatformIO virtual environment from PATH so that rqt_image_view 
# uses the system Python (where PyQt5 is installed) instead of PlatformIO's Python.
export PATH=$(echo $PATH | sed -e 's;/[^:]*\.platformio/penv/bin:;;g')

echo "Launching openni2_camera driver..."
ros2 run openni2_camera openni2_camera_driver &
CAMERA_PID=$!

echo "Waiting for camera to start..."
sleep 3

echo "Opening rqt_image_view..."
ros2 run rqt_image_view rqt_image_view &
RQT_PID=$!

echo "Press [CTRL+C] to stop both the camera node and rqt_image_view."
wait

#!/bin/bash
# Run once on the Raspberry Pi to install all dependencies.
# Usage: bash setup_pi.sh

echo "=== Nextrones Pi Setup ==="

# --- ROS 2 Jazzy ---
if ! command -v ros2 &>/dev/null; then
    echo "[1/7] Installing ROS 2 Jazzy..."
    sudo apt update && sudo apt install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    sudo apt install -y software-properties-common
    sudo add-apt-repository universe -y
    sudo apt update && sudo apt install -y curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list
    sudo apt update
    sudo apt install -y ros-jazzy-ros-base
else
    echo "[1/7] ROS 2 Jazzy already installed."
fi

# --- Nav2 + bridge packages ---
echo "[2/7] Installing ROS packages..."
sudo apt install -y \
    ros-jazzy-nav2-bringup \
    ros-jazzy-depthimage-to-laserscan \
    ros-jazzy-rtabmap-ros \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-vision-opencv \
    python3-colcon-common-extensions \
    python3-rosdep

# --- Kobuki driver ---
echo "[3/7] Installing Kobuki udev rules..."
sudo apt install -y ros-jazzy-kobuki-ros 2>/dev/null || true
# Kobuki USB serial rule
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="kobuki", MODE="0666"' \
    | sudo tee /etc/udev/rules.d/57-kobuki.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# --- Kinect (OpenNI2) ---
echo "[4/7] Installing Kinect drivers..."
sudo apt install -y libfreenect-dev freenect
sudo apt install -y ros-jazzy-openni2-camera 2>/dev/null || true

# --- Python packages ---
echo "[5/7] Installing Python packages..."
pip3 install ultralytics --break-system-packages
pip3 install "numpy<2" --break-system-packages
pip3 install psutil --break-system-packages

# --- Swap file (4GB) ---
echo "[6/7] Setting up swap..."
if [ ! -f /swapfile ]; then
    sudo fallocate -l 4G /swapfile
    sudo chmod 600 /swapfile
    sudo mkswap /swapfile
    sudo swapon /swapfile
    echo '/swapfile swap swap defaults 0 0' | sudo tee -a /etc/fstab
    echo "Swap created."
else
    echo "Swap already exists."
fi

# --- Shell environment ---
echo "[7/7] Configuring shell..."
BASHRC="$HOME/.bashrc"
grep -q "ros/jazzy" "$BASHRC" || echo "source /opt/ros/jazzy/setup.bash" >> "$BASHRC"
grep -q "ros_final" "$BASHRC" || echo "source ~/ros_final/install/setup.bash" >> "$BASHRC"
grep -q "ROS_LOG_DIR" "$BASHRC" || echo "export ROS_LOG_DIR=/dev/shm/ros_logs" >> "$BASHRC"
grep -q "ROS_DOMAIN_ID" "$BASHRC" || echo "export ROS_DOMAIN_ID=0" >> "$BASHRC"

echo ""
echo "=== Setup complete. Reboot recommended. ==="
echo "Next step: run  bash ~/ros_final/scripts/build_pi.sh"

# Duckietown_ROS2_CPS

> Main repository for the Duckietown ROS2 CPS project.

# DB21M/Jetson Nano

## Prepare SD card
Follow the instructions [here](https://github.com/nicholas-gs/dt-image-maker/tree/nano/l4t) to prepare a fresh SD card for the DB21M/jetson nano with ROS2 Foxy installed.

## Prepare directory structure

```bash
cd ~
git clone --branch main https://github.com/nicholas-gs/duckietown_ros2_cps
mv duckietown_ros2_cps/duckietown ~/
# We don't need the cloned repo anymore
rm -r duckietown_ros2_cps/
```

```bash
# append to bashrc
cat ~/duckietown/append_bash >> ~/.bashrc
source ~/.bashrc
```

The files in `~/duckietown/config` are used to store configuration settings for duckietown. Make sure the settings are correct for your robot. After making changes, make sure to source the `~/.bashrc` file again to take effect.

## Install system dependencies

We need the following dependencies to use the hardware on the jetson nano.

```bash
pip3 install wheel

sudo apt install -y \
    i2c-tools \
    v4l-utils \
    jetson-gpio-common \
    python3-jetson-gpio
```

## Setup Permissions

```bash
# usb permissions
sudo usermod -aG dialout $USER

# i2c permissions
sudo usermod -aG i2c $USER

# gpio permissions
sudo groupadd -f -r gpio
sudo usermod -aG gpio $USER

# video permissions
sudo groupadd -f -r video
sudo usermod -aG video $USER

sudo udevadm control --reload-rules && sudo udevadm trigger
```

## OpenCV/Gstreamer

Gstreamer and Opencv should be installed by default. Just make sure that Opencv was compiled with gstreamer support using `cv2.getBuildInformation()`.

## Download the source code for Duckietown

```bash
mkdir -p ~/dt_ws/src
cd ~/dt_ws
wget https://raw.githubusercontent.com/nicholas-gs/duckietown_ros2_cps/main/main.repos
vcs import src < main.repos
```

## Install Duckietown dependencies

Since the L4T image is based off Ubuntu 18.04 which is a not officially supported OS for Foxy, using `rosdep` is likely to cause complications due to depedency chains.

```bash
# Example only, not recommended to run!
rosdep install --from-paths src --ignore-src --rosdistro foxy --os ubuntu:focal -y
```

So instead we install the dependencies manually.
```bash
pip3 install smbus \
    smbus2 \
    transforms3d \
    dataclasses \
    numpy \
    pyserial \
    semver

sudo apt update
# Install the Adafruit PiOLED monochrome OLED driver
cd ~/dt_ws/src/non-ros/installPiOLED
./installPiOLED.sh
```

## Compile Duckietown codebase
```bash
cd ~/dt_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "source ~/dt_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

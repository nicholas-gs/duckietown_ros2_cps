### Download the source code

```bash
mkdir -p ~/dt_ws/src
cd ~/dt_ws
wget https://raw.githubusercontent.com/nicholas-gs/duckietown_ros2_cps/main/main.repos
vcs import src < main.repos
```

### Install dependencies
```bash
cd ~/dt_ws
rosdep install --from-paths src --ignore-src --rosdistro foxy -y
```

### Compiling VL53L0X TOF driver from source
```bash
cd ~/dt_ws/src/thirdparty/VL53L0X_rasp_python
make
```

### Compilation
```bash
cd ~/dt_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Install
Create workspace and clone this repo:

```
source /opt/ros/humble/setup.bash
mkdir -p workspace/ros_ur_driver
cd workspace/ros_ur_driver
git clone -b humble_prueba git@github.com:INGENIA-ETSII-23-24/ingenia_ws.git src
vcs import src --skip-existing --input src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver-not-released.${ROS_DISTRO}.repos
rosdep update
rosdep install --ignore-src --from-paths src -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
Si ocurren fallos durante el colcon build, actualizar sistema operativo:
```
sudo apt update
sudo apt upgrade
```

## Launch the simulation
```
ros2 launch ur_bringup lanzar_simulacion.launch.py 
```


## Launch the real robot
```
ros2 launch ur_bringup lanzar_robot.launch.py 
```


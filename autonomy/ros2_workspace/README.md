# ROS2 Robot Integrations

Topics of interest include :
- Utilizing [Ouster ROS2 drivers](https://github.com/ouster-lidar/ouster-ros/tree/master) for interfacing with OS1 LiDAR sensor
- Leveraging simulation environment using ROS2 suite, [especially for lidar sensing](https://wilselby.com/2019/05/simulating-an-ouster-os-1-lidar-sensor-in-ros-gazebo-and-rviz/)
- Implementing ROS2 packages for point cloud based navigation (SLAM, etc.)

Ideas :
- ROS2 OS1 drivers publish lidar data as ROS messages on [topics](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html) over IP. We would like to write this data to shared memory instead. Here's a related ROS package : [https://wiki.ros.org/shm_transport](https://wiki.ros.org/shm_transport)

## Building dependencies

Guide here : [https://industrial-training-master.readthedocs.io/en/foxy/_source/session1/ros2/2-Installing-Existing-Packages.html](https://industrial-training-master.readthedocs.io/en/foxy/_source/session1/ros2/2-Installing-Existing-Packages.html)

Distro : Humble Hawksbill

Get required packages :

```bash
sudo apt install -y         \
    build-essential         \
    libeigen3-dev           \
    libjsoncpp-dev          \
    libspdlog-dev           \
    libcurl4-openssl-dev    \
    cmake                   \
    python3-colcon-common-extensions
```

### Using apt 

```bash
sudo apt install -y         \
    ros-humble-pcl-ros      \
    ros-humble-tf2-eigen    \
    ros-humble-rviz2
```

### From source

Get required packages above;

Clone required repos into src/ :
```bash
(
    cd src/
    git clone https://github.com/ros-perception/perception_pcl
    git clone --branch humble https://github.com/ros2/geometry2
    git clone --branch humble https://github.com/ros2/rviz
    git clone --branch ros2 https://github.com/ouster-lidar/ouster-ros.git
)
```

run `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release` here then `source install/setup.bash`

## Rebuilding aurora_core

run `colcon build --packages-select aurora_core`

## Launching aurora_core

`ros2 run aurora_core aurora_node_core`

or

`ros2 launch launchfile.py`

or

`./run_aurora_core.sh`

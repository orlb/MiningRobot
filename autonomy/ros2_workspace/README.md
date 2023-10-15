## Building ros2_ouster_drivers and dependencies

Requires packages : 
- libtins
- jsoncpp cmake build
- boost
- pcl

Clone repos
- [pcl_msgs ros2 branch](https://github.com/ros-perception/pcl_msgs.git)
- [perception_pcl](https://github.com/ros-perception/perception_pcl.git)
- [ros2_ouster_drivers](https://github.com/ros-drivers/ros2_ouster_drivers.git)

to src, run `colcon build` here then `source install/setup.bash`

## Rebuilding aurora_core

run `colcon build --packages-select aurora_core`

## Running executable

`ros2 run aurora_core aurora_node_core`

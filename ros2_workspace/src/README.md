## Building ros2_ouster_drivers manually

Requires : 
- libtins
- jsoncpp cmake build
- boost
- pcl

ros2-ouster-drivers and dependencies. build with
`cmake -S . -B build; make && sudo make install`
in this order :
- pcl_msgs
- perception_pcl/pcl_conversions
- ros2_ouster_drivers/ouster_msgs
- ros2_ouster_drivers/ros2_ouster

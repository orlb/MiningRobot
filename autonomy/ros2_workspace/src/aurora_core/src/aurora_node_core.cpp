/*
 |  ROS Node
 |
 |  Test reading shared memory from robot
 |
 *-----------------------*/

#include <rclcpp/rclcpp.hpp>    // ROS2 includes

#include "base/include/aurora/robot_base.h"        // aurora : robot_base
#include "base/include/aurora/robot_states.cpp"    // aurora : state_to_string
#include "base/include/aurora/lunatic.h"           // aurora : MAKE_exchange_backend_state()

#include <iostream>
#include <chrono>
#include <thread>

int main ( int argc, char ** argv ) {

    /* ros2 : node init ------------------------------------------------------------------------- */

        // This must be called before anything else ROS-related
    rclcpp::init(argc, argv);

        // Create a ROS node
    auto node = std::make_shared<rclcpp::Node>("aurora_node_core");

    /* ~ros2 ------------------------------------------------------------------------------------ */

    /* main loop : read state from robot_base --------------------------------------------------- */

        // get robot_base
    MAKE_exchange_backend_state();
    const robot_base & robot = exchange_backend_state.read();

    while ( true ) {
        std::cout << state_to_string(robot.state) << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        // RCLCPP_INFO(node->get_logger(), "This works!");
    }

    /* ~main loop ------------------------------------------------------------------------------- */

        // Don't exit the program.
    //rclcpp::spin(node);
    
    return 0;
}


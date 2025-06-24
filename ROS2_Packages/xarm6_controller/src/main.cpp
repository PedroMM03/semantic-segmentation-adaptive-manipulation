/*
 * Project name: Semantic Segmentation for Adaptive Manipulation with Industrial Robots
 * Author: Pedro Martin
 * Electronics, Robotics and Mechatronics Engineering - University of Malaga
 * Date: 2025
 */

#include "xarm6_controller/xarm6_controller.hpp"
#include "rclcpp/rclcpp.hpp"

#include <thread>
#include <termios.h>
#include <unistd.h>
#include <iostream>

int main(int argc, char *argv[])
{
    // Initialize ROS 2 communication
    rclcpp::init(argc, argv);

    // Create an instance of the XarmController node
    auto node = std::make_shared<XarmController>();

    // Set the loop rate to 5 Hz (5 cycles per second)
    rclcpp::Rate rate(5);

    // Main loop runs while ROS 2 is still running
    while (rclcpp::ok())
    {
        // Process incoming ROS messages and callbacks without blocking
        rclcpp::spin_some(node);

        // Call the main logic function of the XarmController
        node->main_logic();

        // Sleep to maintain the loop rate at 5 Hz
        rate.sleep();
    }

    // If the loop exits, enter a blocking spin to keep node alive until shutdown
    rclcpp::spin(node);

    // Cleanly shutdown ROS 2 when finished
    rclcpp::shutdown();

    return 0;
}

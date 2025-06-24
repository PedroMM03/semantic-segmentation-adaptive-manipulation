/*
 * Project name: Semantic Segmentation for Adaptive Manipulation with Industrial Robots
 * Author: Pedro Martin
 * Electronics, Robotics and Mechatronics Engineering - University of Malaga
 * Date: 2025
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "xarm6_controller/srv/process_object.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/float32.hpp"

// XARM_MSGS SERVICES - Services and messages used to control the xArm robot
#include "xarm_msgs/srv/set_int16.hpp"
#include "xarm_msgs/srv/call.hpp"
#include "xarm_msgs/srv/move_cartesian.hpp"
#include "xarm_msgs/msg/robot_msg.hpp"
#include "xarm_msgs/srv/vacuum_gripper_ctrl.hpp"
#include "xarm_msgs/srv/set_int16_by_id.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/float32.hpp"
#include <stdio.h>
#include <stdlib.h>
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_ros/transform_listener.h"
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "Eigen/Geometry"
#include <vector>
#include <cmath>
#include <array>
#include <string>


class XarmController : public rclcpp::Node
{
public:

    // Constructor and destructor
    XarmController();
    ~XarmController();

    // Main function that handles the core processing and decision-making logic for robot control
    void main_logic();

    // Call the xArm driver service to set the robot mode (e.g. manual, automatic)
    void set_robot_mode(int mode);

    // Call the xArm driver service to set the robot state (e.g. ready, running, stopped)
    void set_robot_state(int state);

    // Call the xArm driver service to clear any errors in the robot controller
    void clean_robot_errors();

    // Send a Cartesian target pose to the robot, including optional speed and acceleration parameters
    void send_target_pose(const std::array<float,6>& target_pose, float speed = 30.0, float acc = 200.0);

    // Check if the current robot pose is within specified positional and angular tolerances compared to a target pose
    bool check_robot_pose(const std::array<float,6>& robot_pose, const std::array<float,6>& target_pose, float pos_tolerance, float ang_tolerance);

    // Call the xArm driver service to enable or disable the vacuum gripper
    void set_vacuum_gripper(bool on);

    // Call the xArm driver service to enable robot motion
    void set_motion_enable();

    // Service server to handle requests for processing objects, called by other nodes or clients
    rclcpp::Service<xarm6_controller::srv::ProcessObject>::SharedPtr server_process;

    // Callback function to handle the ProcessObject service requests
    void handle_process_service(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<xarm6_controller::srv::ProcessObject::Request> request,
        const std::shared_ptr<xarm6_controller::srv::ProcessObject::Response> response);

    // Service clients for calling various xArm services (movement, mode, state, errors, gripper control, motion enable)
    rclcpp::Client<xarm_msgs::srv::MoveCartesian>::SharedPtr robot_move_client;
    rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedPtr set_state_client;
    rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedPtr set_mode_client;
    rclcpp::Client<xarm_msgs::srv::Call>::SharedPtr clean_error_client;
    rclcpp::Client<xarm_msgs::srv::VacuumGripperCtrl>::SharedPtr set_vacuum_gripper_client;
    rclcpp::Client<xarm_msgs::srv::SetInt16ById>::SharedPtr motion_enable_client;

    // Flag indicating whether the controller is ready to capture data or process commands
    bool ready_to_capture;

    // TF2 objects for transforming coordinates between different frames (camera, robot base, TCP, etc.)
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

private:

    // Publisher to publish the current step of the processing pipeline or state machine (int16 message)
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr process_step_pub;

    // Subscribers to receive data from topics:
    // - Grasp point position in the camera frame
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr grasp_position_sub;

    // - Grasp orientation vector in the camera frame
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr grasp_orientation_sub;

    // - Robot state messages from the xArm driver
    rclcpp::Subscription<xarm_msgs::msg::RobotMsg>::SharedPtr robot_states_sub;

    // - Boolean flag to check if the object is detected or verified
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr check_object_sub;

    // Callback functions for each subscriber, handling incoming messages
    void grasp_position_cb(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void grasp_orientation_cb(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
    void robot_states_cb(const xarm_msgs::msg::RobotMsg::SharedPtr msg);
    void check_object_cb(const std_msgs::msg::Bool::SharedPtr msg);

    // ----------- Variables for robot movement and coordinate transformations -----------

    // Grasp point and normal vector in camera coordinate frame
    Eigen::Vector4d grasp_point_cam;
    Eigen::Vector3d normal_vector_cam;

    // Grasp point and normal vector transformed into the robot base coordinate frame
    Eigen::Vector4d grasp_point_base;
    Eigen::Vector3d normal_vector_base;

    // Approach point for the grasp, expressed in robot base frame
    Eigen::Vector4d approach_point_base;

    // Roll-pitch-yaw orientation vector
    Eigen::Vector3d rpy;

    // Transformations between robot TCP (tool center point) and base, and camera to TCP frames
    Eigen::Affine3d T_tcp_to_base;
    Eigen::Affine3d T_cam_to_tcp;

    // Offset to modify grasp position during a retry attempt
    float offset;

    // Robot poses expressed as arrays of 6 floats (x,y,z position + roll, pitch, yaw orientation)
    std::array<float,6> robot_pose;   // Current pose of the robot from driver feedback
    std::array<float,6> target_pose;  // Target pose for the robot to move to
    std::array<float,6> home_pose;    // Initial pose to start manipulation tasks

    // Selected object information, coming from segmentation or perception nodes
    int obj_id;                      // ID of the selected object
    std::string obj_class_name;      // Class name of the selected object
    Eigen::Vector4d approach_point_cam;  // Approach point in camera coordinate frame
    bool checked_object;             // Flag indicating whether the object has been detected and verified

    // -------- Flow control variables for the robot operation --------

    std_msgs::msg::Int16 process_step;    // Current step or phase of the processing pipeline
    int robot_state;                      // Current state of the robot
    int last_robot_state;                 // Previous state to detect changes
    bool normal_received;                 // Flag indicating if a normal vector has been received
    bool point_received;                  // Flag indicating if grasp point data has been received
    bool target_pose_sent;                // Flag to avoid changing target pose until current one is reached
    int max_wait_cycles;                  // Maximum number of cycles to wait for data or conditions
    int wait_counter;                    // Current wait cycle counter
    std_msgs::msg::Int16 get_orientation_trigger_ID;  // Identifier for triggering orientation retrieval
    bool get_orientation_trigger_Flag;   // Flag to enable orientation retrieval
    bool check_received;                  // Flag to track if object check message has been received
    int pick_attempt;                    // Counter for number of pick attempts made
    int max_pick_attempts;               // Maximum allowed pick attempts before giving up
    double ang_tolerance;       // Angular tolerance for pose alignment (in radians)
    double pos_tolerance;       // Positional tolerance for grasping (in milimeters)
};

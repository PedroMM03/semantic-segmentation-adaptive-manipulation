
/*
 * Project name: Semantic Segmentation for Adaptive Manipulation with Industrial Robots
 * Author: Pedro Martin
 * Electronics, Robotics and Mechatronics Engineering - University of Malaga
 * Date: 2025
 */

#include "xarm6_controller/xarm6_controller.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

// Constructor of the XarmController node
XarmController::XarmController() : Node("xarm6_controller")
{
    // QoS profile setup for publishers and subscribers
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliable();         // Ensure reliable message delivery
    qos.transient_local();  // Transient local to receive late joiners

    // Initialize publisher to publish the current process step of the controller
    process_step_pub = this->create_publisher<std_msgs::msg::Int16>(
        "/xarm6_controller/process_step", qos);
    
    // Initialize subscriptions to receive object position, orientation, robot state and object check
    grasp_position_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/zed_vision/object_position", qos, std::bind(&XarmController::grasp_position_cb, this, _1));
    grasp_orientation_sub = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
        "/zed_vision/object_orientation", qos, std::bind(&XarmController::grasp_orientation_cb, this, _1));
    robot_states_sub = this->create_subscription<xarm_msgs::msg::RobotMsg>(
        "/xarm/robot_states", 10, std::bind(&XarmController::robot_states_cb, this, _1));
    check_object_sub = this->create_subscription<std_msgs::msg::Bool>(
        "/zed_vision/check_object", qos, std::bind(&XarmController::check_object_cb, this, _1));

    // Create a service server to handle process requests from clients
    server_process = this->create_service<xarm6_controller::srv::ProcessObject>(
        "process_object_service",
        std::bind(&XarmController::handle_process_service, this, _1, _2, _3));

    // Initialize movement-related variables with default values
    normal_vector_base << 0.0, 0.0, -1.0;  // Default normal vector in robot base frame
    normal_vector_cam << 0.0, 0.0, -1.0;   // Default normal vector in camera frame
    approach_point_base << 0.0, 0.0, 0.0, 1;  // Homogeneous coordinates for approach point
    robot_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};   // Current robot pose initialization
    target_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // Target pose initialization
    grasp_point_cam << 0.0, 0.0, 0.0, 1;    // Grasp point in camera coordinates (homogeneous)
    grasp_point_base << 0.0, 0.0, 0.0, 1;   // Grasp point in robot base coordinates (homogeneous)
    rpy << 0.0, 0.0, 0.0;                    // Roll, pitch, yaw initialization
    this->declare_parameter<std::vector<double>>("home_pose", {0.213, -0.029, 0.357, 3.14, -0.33, 0.003});
    std::vector<double> home_pose_vec = this->get_parameter("home_pose").as_double_array(); // Defined home position for the robot
    // Casting std::array<float, 6>
    for (size_t i = 0; i < 6; ++i) {
        home_pose[i] = static_cast<float>(home_pose_vec[i]);
    }
     
    offset = 0;                              // Offset for adjustment in grasp attempts

    // Initialize transformation matrices as identity matrices
    T_tcp_to_base = Eigen::Affine3d::Identity();
    T_cam_to_tcp = Eigen::Affine3d::Identity();

    // Define a rotation of 90 degrees around Z-axis for camera to TCP transformation
    Eigen::Matrix3d rotation;
    rotation << 0, -1, 0,
                1,  0, 0,
                0,  0, 1;
    T_cam_to_tcp.linear() = rotation;

    // Define translation vector from camera to TCP in meters
    T_cam_to_tcp.translation() << 0.071, -0.023, 0.0;

    // Initialize logic control variables
    process_step.data = 0;      // Starting process step
    max_wait_cycles = 300;      // Maximum wait cycles for topic callbacks
    robot_state = 0;            // Initial robot state
    last_robot_state = 0;       // Store previous robot state
    get_orientation_trigger_ID.data = -1;   // Invalid trigger ID at start
    get_orientation_trigger_Flag = false;   // Trigger flag not set initially
    wait_counter = 0;           // Counter for waiting loops
    ready_to_capture = false;   // Flag for readiness to capture new data
    normal_received = false;    // Flag indicating normal vector data received
    point_received = false;     // Flag indicating grasp point data received
    check_received = false;     // Flag indicating object check data received
    target_pose_sent = false;   // Flag to avoid sending multiple target poses
    pick_attempt = 0;           // Count of pick attempts
    this->declare_parameter("max_pick_attempts", 3);
    this->get_parameter("max_pick_attempts", max_pick_attempts);    // Maximum allowed pick attempts
    this->declare_parameter("pos_tolerance", 1.0);
    this->get_parameter("pos_tolerance", pos_tolerance);  // Positional tolerance for grasping (in milimeters)
    this->declare_parameter("ang_tolerance", 0.3);       
    this->get_parameter("ang_tolerance", ang_tolerance); // Angular tolerance for pose alignment (in radians)
    // Initialize selected object data
    obj_id = 0;
    obj_class_name = "";
    approach_point_cam << 0.0, 0.0, 0.0, 1;
    checked_object = false;

    // Initialize service clients to communicate with xArm driver services
    set_mode_client = this->create_client<xarm_msgs::srv::SetInt16>("/xarm/set_mode");
    set_state_client = this->create_client<xarm_msgs::srv::SetInt16>("/xarm/set_state");
    robot_move_client = this->create_client<xarm_msgs::srv::MoveCartesian>("/xarm/set_position");
    clean_error_client = this->create_client<xarm_msgs::srv::Call>("/xarm/clean_error");
    set_vacuum_gripper_client = this->create_client<xarm_msgs::srv::VacuumGripperCtrl>("/xarm/set_vacuum_gripper");
    motion_enable_client = this->create_client<xarm_msgs::srv::SetInt16ById>("/xarm/motion_enable");

    // Initialize tf2 buffer and listener to handle coordinate frame transforms
    tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
}

// Destructor of the XarmController node
XarmController::~XarmController()
{
    RCLCPP_INFO(this->get_logger(), "xarm6_controller destroyed");
    // Turn off the vacuum gripper when destroying the node to ensure safe shutdown
    set_vacuum_gripper(false);
}


void XarmController::handle_process_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<xarm6_controller::srv::ProcessObject::Request> request,
    const std::shared_ptr<xarm6_controller::srv::ProcessObject::Response> response
)
{
    // Log the received request data: object ID, class name, and approximate position
    RCLCPP_INFO(this->get_logger(), 
                "Received object request: ID=%d, Class='%s', Position=(%.3f, %.3f, %.3f)",
                request->id,
                request->class_name.c_str(),
                request->aprox_x,
                request->aprox_y,
                request->aprox_z);

    // Set positive response to acknowledge processing start
    response->success = true;
    response->message = "Object is being processed ... ";

    // Store the requested object information internally for further processing
    obj_id = request->id;
    obj_class_name = request->class_name.c_str();
    approach_point_cam = Eigen::Vector4d(request->aprox_x, request->aprox_y, request->aprox_z, 1.0);

    // Trigger the process by updating and publishing the process step
    process_step.data = 2;
    process_step_pub->publish(process_step);
}





void XarmController::main_logic()
{
    

    RCLCPP_INFO(this->get_logger(), "Process Step: %d",process_step.data);
    RCLCPP_INFO(this->get_logger(), "Robot state: %d",robot_state);
    RCLCPP_INFO(this->get_logger(), "Grasp Point Z: %f",grasp_point_base(2));
    
    
    //Initialize robot: motieon_enable, close vacuum, move to home position -- param
    

    switch (process_step.data)
    {
        case 0: // Initialize
        {
            
            if(!target_pose_sent)
            {
                RCLCPP_INFO(this->get_logger(), "Initializ xarm6_controller_node ...");
                normal_vector_base << 0.0,0.0,-1.0;
                normal_vector_cam<<0.0,0.0,-1.0;
                approach_point_base << 0.0,0.0,0.0,1;
                target_pose={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                grasp_point_cam << 0.0,0.0,0.0,1;
                grasp_point_base << 0.0,0.0,0.0,1;
                rpy <<0.0,0.0,0.0;
                // Motion enable
                set_motion_enable();
                // Deactivate vacuum gripper
                set_vacuum_gripper(false);
    
                //Set robot to home pose
                send_target_pose(home_pose);


            }else{

                //Target pose has been sent: now it must be checked whether it is reached
                if(robot_state==0 || robot_state==2) //Robot is idle
                {
                        
                        if (check_robot_pose(robot_pose,home_pose,1.0,0.3))  //Check robot reached target pose
                        {
                            RCLCPP_INFO(this->get_logger(), "Home Pose reached");
                            
                            //Setting up for next step
                            process_step.data=1; //Next step: get grasp orientation
                            process_step_pub->publish(process_step);
                            target_pose_sent=false;
                            set_vacuum_gripper(false);
                            rclcpp::sleep_for(std::chrono::milliseconds(8000));

                        }else{
                            RCLCPP_INFO(this->get_logger(), "Home Pose not reached");
                            //Call service movement of the manipulator robot
                            //Move robot: Needs position in m
                            //clean_robot_errors();
                            send_target_pose(home_pose);

                        }
                

                }else if(robot_state==4)
                {
                    // Error state
                    clean_robot_errors();
                    set_robot_mode(0);
                    set_robot_state(0);
                }  

            } 
            
        }
        break;

        case 1: //Waiting

        {
            RCLCPP_INFO(this->get_logger(), "Waiting...");

            
        }
        break;

        case 2: // Object Approach
        {
            if (!target_pose_sent)
            {
                try {
                    rclcpp::Time now = this->get_clock()->now();

                    // Get the latest TCP → Base transform
                    geometry_msgs::msg::TransformStamped tcp_to_base_tf =
                        tf_buffer->lookupTransform("link_base", "link6", now, rclcpp::Duration::from_seconds(0.2));

                    T_tcp_to_base = tf2::transformToEigen(tcp_to_base_tf);
                    Eigen::Affine3d T_cam_to_base = T_tcp_to_base * T_cam_to_tcp;

                    // Show approach point in camera frame
                    RCLCPP_INFO(this->get_logger(), "Approach Point in Camera: [%.3f, %.3f, %.3f, %.3f]",
                        approach_point_cam(0), approach_point_cam(1), approach_point_cam(2), approach_point_cam(3));

                    // Apply Camera → Base transformation in a single step
                    approach_point_base = T_cam_to_base.matrix() * approach_point_cam;

                    RCLCPP_INFO(this->get_logger(), "Approach Point in Base: [%.3f, %.3f, %.3f]",
                        approach_point_base(0), approach_point_base(1), approach_point_base(2));

                    // Add offset to improve object visibility from camera
                    target_pose = {
                        static_cast<float>(approach_point_base(0) - 0.07),
                        static_cast<float>(approach_point_base(1)),
                        static_cast<float>(approach_point_base(2) + 0.20),
                        3.14, 0.0,0.0
                    };
                    send_target_pose(target_pose);

                } catch (tf2::TransformException &ex) {
                    RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
                }

            } else {
                // Target pose was already sent: check if robot reached it
                if (robot_state == 0 || robot_state == 2) // Robot is idle
                {
                    rclcpp::sleep_for(std::chrono::milliseconds(1000));

                    if (check_robot_pose(robot_pose, target_pose, 1.0, 0.3)) // Verify target pose reached
                    {
                        RCLCPP_INFO(this->get_logger(), "Approach Point reached");

                        // Prepare next step
                        process_step.data = 3;
                        process_step_pub->publish(process_step);
                        target_pose_sent = false;
                        wait_counter = 0;

                        rclcpp::sleep_for(std::chrono::milliseconds(8000)); // Allow settling

                    } else {
                        RCLCPP_INFO(this->get_logger(), "Approach Point not reached");

                        // Retry sending target pose
                        send_target_pose(target_pose);
                    }

                } else if (robot_state == 4) {
                    // Robot error state
                    RCLCPP_ERROR(this->get_logger(), "Target position is unreachable");
                    clean_robot_errors();
                    set_robot_mode(0);
                    set_robot_state(0);
                }
            }
        }
        break;


        


        case 3: // Get grasp point and orientation
        {
            RCLCPP_WARN(this->get_logger(), "Waiting Grasp Pose");

            // Proceed only if both grasp point and normal vector were received
            if (point_received && normal_received)
            {
                try {
                    rclcpp::Time now = this->get_clock()->now();
                    
                    // Get the latest TCP → Base transform
                    geometry_msgs::msg::TransformStamped tcp_to_base_tf =
                        tf_buffer->lookupTransform("link_base", "link6", now, rclcpp::Duration::from_seconds(0.2));

                    T_tcp_to_base = tf2::transformToEigen(tcp_to_base_tf);
                    Eigen::Affine3d T_cam_to_base = T_tcp_to_base * T_cam_to_tcp;

                    // Transform grasp point to base frame
                    grasp_point_base = T_cam_to_base.matrix() * grasp_point_cam;
                    RCLCPP_INFO(this->get_logger(), "Grasp Point in Base: [%.3f, %.3f, %.3f]",
                                grasp_point_base(0), grasp_point_base(1), grasp_point_base(2));

                    // Transform normal vector to base frame
                    normal_vector_base = T_cam_to_base.linear() * normal_vector_cam;
                    RCLCPP_INFO(this->get_logger(), "Normal in Base: [%.3f, %.3f, %.3f]",
                                normal_vector_base(0), normal_vector_base(1), normal_vector_base(2));

                    // Define desired Z-axis (grasp approach direction)
                    Eigen::Vector3d z_desired = -normal_vector_base.head<3>();
                    RCLCPP_INFO(this->get_logger(), "z DESIRED: [%.3f, %.3f, %.3f]",
                                z_desired(0), z_desired(1), z_desired(2));

                    Eigen::Vector3d z_axis = z_desired.normalized();
                    Eigen::Vector3d x_hint(1.0, 0.0, 0.0);

                    // Use alternative hint if vectors are nearly parallel
                    if (fabs(z_axis.dot(x_hint)) > 0.95) {
                        x_hint = Eigen::Vector3d(0.0, 1.0, 0.0);
                    }

                    // Compute orthonormal frame (X, Y, Z)
                    Eigen::Vector3d x_axis = (x_hint - z_axis.dot(x_hint) * z_axis).normalized();
                    Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();

                    // Build rotation matrix from frame
                    Eigen::Matrix3d TCP_rotation;
                    TCP_rotation.col(0) = x_axis;
                    TCP_rotation.col(1) = y_axis;
                    TCP_rotation.col(2) = z_axis;

                    // Extract RPY angles (roll, pitch, yaw)
                    rpy = TCP_rotation.eulerAngles(0, 1, 2);

                    // Normalize angles to [-π, π)
                    for (int i = 0; i < 3; ++i) {
                        rpy(i) = std::atan2(std::sin(rpy(i)), std::cos(rpy(i)));
                    }

                    RCLCPP_INFO(this->get_logger(), "RPY final: [%.3f, %.3f, %.3f]",
                                rpy(0), rpy(1), rpy(2));

                    // Move to next processing step
                    process_step.data = 4;
                    process_step_pub->publish(process_step);
                    point_received = false;
                    normal_received = false;
                    wait_counter = 0;

                } catch (tf2::TransformException &ex) {
                    RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
                }

            } else if (wait_counter < max_wait_cycles) {
                wait_counter++;
            } else {
                RCLCPP_INFO(this->get_logger(), "Grasp Position not received");
                wait_counter = 0;
            }
        }
        break;


        

        case 4: // Above object (not oriented)
        {
            if (!target_pose_sent)
            {
                target_pose={static_cast<float>(grasp_point_base(0)), static_cast<float>(grasp_point_base(1)),static_cast<float>(grasp_point_base(2)+0.20) , 3.14, 0.0, 0.0};
                send_target_pose(target_pose,10.0,20.0);

            }else{
                //Target pose has been sent: now it must be checked whether it is reached
                if(robot_state==0 || robot_state==2) //Robot is idle
                {
                        rclcpp::sleep_for(std::chrono::milliseconds(1000));
                        if (check_robot_pose(robot_pose,target_pose,1.0,0.3))  //Check robot reached target pose
                        {
                            RCLCPP_INFO(this->get_logger(), "Target Pose reached");
                            rclcpp::sleep_for(std::chrono::milliseconds(3000));
                            //Setting up for next step
                            process_step.data=5; //Next step: get grasp orientation
                            process_step_pub->publish(process_step);
                            target_pose_sent=false;
                            wait_counter = 0;

                        }
                        
                        else{
                            RCLCPP_INFO(this->get_logger(), "Target Pose not reached");
                            //Call service movement of the manipulator robot
                            //Move robot: Needs position in m
                            //clean_robot_errors();

                            send_target_pose(target_pose,10.0,20.0);

                        }
                        
                

                }else if(robot_state==4)
                {
                    // Error state
                    RCLCPP_ERROR(this->get_logger(), "Target position is unreachable");
                    clean_robot_errors();
                    set_robot_mode(0);
                    set_robot_state(0);
                    
                    
                }  
            }
        }
        break;

        case 5: // Above object (oriented)
        {
            if (!target_pose_sent)
            {
                target_pose={static_cast<float>(grasp_point_base(0)), static_cast<float>(grasp_point_base(1)),static_cast<float>(grasp_point_base(2)+0.20) , static_cast<float>(rpy(0)), static_cast<float>(-rpy(1)), static_cast<float>(rpy(2))};
                send_target_pose(target_pose);

            }else{
                //Target pose has been sent: now it must be checked whether it is reached
                if(robot_state==0 || robot_state==2) //Robot is idle
                {
                        rclcpp::sleep_for(std::chrono::milliseconds(1000));
                        if (check_robot_pose(robot_pose,target_pose,1.0,0.3))  //Check robot reached target pose
                        {
                            RCLCPP_INFO(this->get_logger(), "Target Pose reached");
                            rclcpp::sleep_for(std::chrono::milliseconds(3000));
                            //Setting up for next step
                            process_step.data=6; //Next step: get grasp orientation
                            process_step_pub->publish(process_step);
                            target_pose_sent=false;
                            wait_counter = 0;

                        }
                        
                        else{

                            RCLCPP_INFO(this->get_logger(), "Target Pose not reached");

                            send_target_pose(target_pose);

                        }
                        
                

                }else if(robot_state==4)
                {
                    // Error state
                    RCLCPP_ERROR(this->get_logger(), "Target position is unreachable");
                    clean_robot_errors();
                    set_robot_mode(0);
                    set_robot_state(0);
                    
                    
                }  
            }
        }
        break;

        case 6: // Picking object
        {
            if (!target_pose_sent)
            {
                target_pose={static_cast<float>(grasp_point_base(0)), static_cast<float>(grasp_point_base(1)),static_cast<float>(grasp_point_base(2)+0.115-offset) , static_cast<float>(rpy(0)), static_cast<float>(-rpy(1)), static_cast<float>(rpy(2))};
                send_target_pose(target_pose,5.0,20.0);

            }else{
                //Target pose has been sent: now it must be checked whether it is reached
                if(robot_state==0 || robot_state==2) //Robot is idle
                {
                        rclcpp::sleep_for(std::chrono::milliseconds(1000));
                        if (check_robot_pose(robot_pose,target_pose,1.0,0.3))  //Check robot reached target pose
                        {
                            RCLCPP_INFO(this->get_logger(), "Target Pose reached");
                            //Setting up for next step
                            process_step.data=7; //Next step: get grasp orientation
                            process_step_pub->publish(process_step);
                            target_pose_sent=false;
                            wait_counter = 0;
                            set_vacuum_gripper(true);
                            rclcpp::sleep_for(std::chrono::milliseconds(3000));

                        }
                        
                        else{
                            RCLCPP_INFO(this->get_logger(), "Target Pose not reached");
                           

                            send_target_pose(target_pose,5.0,20.0);

                        }
                        
                

                }else if(robot_state==4)
                {
                    // Error state
                    RCLCPP_ERROR(this->get_logger(), "Target position is unreachable");
                    clean_robot_errors();
                    set_robot_mode(0);
                    set_robot_state(0);

                    
                }  
            }
        }
        break;

        case 7: // Lifting object
        {
            if (!target_pose_sent)
            {
                target_pose={static_cast<float>(grasp_point_base(0)), static_cast<float>(grasp_point_base(1)),static_cast<float>(grasp_point_base(2)+0.20) , static_cast<float>(rpy(0)), static_cast<float>(-rpy(1)), static_cast<float>(rpy(2))};
                send_target_pose(target_pose);

            }else{
                //Target pose has been sent: now it must be checked whether it is reached
                if(robot_state==0 || robot_state==2) //Robot is idle
                {
                        rclcpp::sleep_for(std::chrono::milliseconds(1000));
                        if (check_robot_pose(robot_pose,target_pose,1.0,0.3))  //Check robot reached target pose
                        {
                            RCLCPP_INFO(this->get_logger(), "Target Pose reached");
                            rclcpp::sleep_for(std::chrono::milliseconds(3000));
                            //Setting up for next step
                            process_step.data=8; //Next step: get grasp orientation
                            process_step_pub->publish(process_step);
                            target_pose_sent=false;
                            wait_counter = 0;

                        }
                        
                        else{

                            RCLCPP_INFO(this->get_logger(), "Target Pose not reached");

                            send_target_pose(target_pose);

                        }
                        
                

                }else if(robot_state==4)
                {
                    // Error state
                    RCLCPP_ERROR(this->get_logger(), "Target position is unreachable");
                    clean_robot_errors();
                    set_robot_mode(0);
                    set_robot_state(0);

                    
                }  
            }
        }
        break;

        case 8: // Moving to verification point (approach point from step 2)
        {
            if (!target_pose_sent)
            {
                target_pose={static_cast<float>(approach_point_base(0)-0.07), static_cast<float>(approach_point_base(1)), static_cast<float>(approach_point_base(2)+0.20), 3.14, 0.0,0.0};
                send_target_pose(target_pose);

            }else{
                //Target pose has been sent: now it must be checked whether it is reached
                if(robot_state==0 || robot_state==2) //Robot is idle
                {
                        rclcpp::sleep_for(std::chrono::milliseconds(1000));
                        if (check_robot_pose(robot_pose,target_pose,1.0,0.3))  //Check robot reached target pose
                        {
                            RCLCPP_INFO(this->get_logger(), "Target Pose reached");
                            //Setting up for next step
                            process_step.data=9; //Next step: Check whether object has been picked up
                            process_step_pub->publish(process_step);
                            target_pose_sent=false;
                            wait_counter = 0;
                            rclcpp::sleep_for(std::chrono::milliseconds(8000));

                        }
                        
                        else{
                            RCLCPP_INFO(this->get_logger(), "Target Pose not reached");
                            

                            send_target_pose(target_pose);

                        }
                        
                

                }else if(robot_state==4)
                {
                    // Error state
                    RCLCPP_ERROR(this->get_logger(), "Target position is unreachable");
                    clean_robot_errors();
                    set_robot_mode(0);
                    set_robot_state(0);

                    
                }  
            }
        }
        break;

        case 9: // Verifying grasp
        {
            
            if(check_received)
            {


                if(checked_object)
                {
                    pick_attempt++;
                    if (pick_attempt == max_pick_attempts)
                    {
                        // Object has not been picked: retry
                        process_step.data=0; //Next step: Initialization (give up)
                        process_step_pub->publish(process_step);
                        set_vacuum_gripper(false);
                        target_pose_sent=false;
                        check_received = false;
                        wait_counter = 0;
                        RCLCPP_INFO(this->get_logger(), "Picking object was not possible");
                        rclcpp::sleep_for(std::chrono::milliseconds(1000));
                    }
                    else{
                        // Object has not been picked: retry
                        process_step.data=3; //Next step: Get grasp point and orientation (again)
                        process_step_pub->publish(process_step);
                        set_vacuum_gripper(false);
                        target_pose_sent=false;
                        check_received = false;
                        wait_counter = 0;
                        RCLCPP_INFO(this->get_logger(), "Object not picked");
                        // Se puede poner contador de intentos, hace que baje unos mm mas
                        offset = offset + 0.005;
                        
                        rclcpp::sleep_for(std::chrono::milliseconds(8000));
                    }
                    



                }else{

                    // Object has been picked: continue
                    process_step.data=10; //Next step: Go to destination
                    process_step_pub->publish(process_step);
                    target_pose_sent=false;
                    check_received = false;
                    pick_attempt=0;
                    offset = 0;
                    wait_counter = 0;

                    // Se puede poner contador de intentos, hace que baje unos mm mas
                    rclcpp::sleep_for(std::chrono::milliseconds(1000));
                      
                }
            }else if (wait_counter < max_wait_cycles)
            {
                wait_counter++;
            }else
            {
                RCLCPP_INFO(this->get_logger(), "Check not received");
                wait_counter = 0;
            }
            
            
            
        }
        break;

        case 10: // Moving above destination
        {
            if (!target_pose_sent)
            {
                // Assign a destination point based on the class (for experiment)
                if (obj_class_name == "apple")
                {
                    target_pose={0.15,0.55, 0.20 , 3.14,0.0,0.0};

                }else if (obj_class_name == "cell phone")
                {
                    target_pose={0.60,0.18, 0.20 , 3.14,0.0,0.0};

                }else if (obj_class_name == "banana")
                {
                    target_pose={0.28,0.55, 0.20 , 3.14,0.0,0.0};
                }
                target_pose[2] += 0.20;
                send_target_pose(target_pose);

            }else{
                //Target pose has been sent: now it must be checked whether it is reached
                if(robot_state==0 || robot_state==2) //Robot is idle
                {
                        rclcpp::sleep_for(std::chrono::milliseconds(1000));
                        if (check_robot_pose(robot_pose,target_pose,1.0,0.3))  //Check robot reached target pose
                        {
                            RCLCPP_INFO(this->get_logger(), "Target Pose reached");
                            //Setting up for next step
                            process_step.data=11; //Next step: Leave object in destination point
                            process_step_pub->publish(process_step);
                            target_pose_sent=false;
                            wait_counter = 0;
                            rclcpp::sleep_for(std::chrono::milliseconds(1000));
                        }
                        
                        else{

                            RCLCPP_INFO(this->get_logger(), "Target Pose not reached");

                            send_target_pose(target_pose);

                        }
                        
                

                }else if(robot_state==4)
                {
                    // Error state
                    RCLCPP_ERROR(this->get_logger(), "Target position is unreachable");
                    clean_robot_errors();
                    set_robot_mode(0);
                    set_robot_state(0);

                    

                } 
            } 
        }
        break;

        case 11: // Placing object at destination
        {
            if (!target_pose_sent)
            {
                target_pose[2] -= 0.20;
                send_target_pose(target_pose);

            }else{
                //Target pose has been sent: now it must be checked whether it is reached
                if(robot_state==0 || robot_state==2) //Robot is idle
                {
                        rclcpp::sleep_for(std::chrono::milliseconds(1000));
                        if (check_robot_pose(robot_pose,target_pose,1.0,0.3))  //Check robot reached target pose
                        {
                            RCLCPP_INFO(this->get_logger(), "Target Pose reached");
                            //Setting up for next step
                            process_step.data=12; //Next step: Check whether object has been picked up
                            process_step_pub->publish(process_step);
                            target_pose_sent=false;
                            wait_counter = 0;
                            set_vacuum_gripper(false);
                            rclcpp::sleep_for(std::chrono::milliseconds(2000));
                        }
                        
                        else{

                            RCLCPP_INFO(this->get_logger(), "Target Pose not reached");

                            send_target_pose(target_pose);

                        }
                        
                

                }else if(robot_state==4)
                {
                    // Error state
                    RCLCPP_ERROR(this->get_logger(), "Target position is unreachable");
                    clean_robot_errors();
                    set_robot_mode(0);
                    set_robot_state(0);

                    

                }  
            } 
        }
        break;

        case 12: // Object placed, returning to home
        {
            if (!target_pose_sent)
            {
                target_pose[2] += 0.20;
                send_target_pose(target_pose);

            }else{
                //Target pose has been sent: now it must be checked whether it is reached
                if(robot_state==0 || robot_state==2) //Robot is idle
                {
                        rclcpp::sleep_for(std::chrono::milliseconds(1000));
                        if (check_robot_pose(robot_pose,target_pose,1.0,0.3))  //Check robot reached target pose
                        {
                            RCLCPP_INFO(this->get_logger(), "Target Pose reached");
                            //Setting up for next step
                            process_step.data=0; //Next step: Check whether object has been picked up
                            process_step_pub->publish(process_step);
                            target_pose_sent=false;
                            wait_counter = 0;
                            rclcpp::sleep_for(std::chrono::milliseconds(1000));
                        }
                        
                        else{

                            RCLCPP_INFO(this->get_logger(), "Target Pose not reached");

                            send_target_pose(target_pose);

                        }
                        
                

                }else if(robot_state==4)
                {
                    // Error state
                    RCLCPP_ERROR(this->get_logger(), "Target position is unreachable");
                    clean_robot_errors();
                    set_robot_mode(0);
                    set_robot_state(0);

                    
                }  
            } 
        }
        break;

        
    }
    

}

void XarmController::set_motion_enable()
{
    // Create a request to enable motion on the robot (ID = 8, data = 1 means enable)
    auto request = std::make_shared<xarm_msgs::srv::SetInt16ById::Request>();
    request->id = 8;
    request->data = 1;

    // Wait for the service to be available before sending the request
    while (!motion_enable_client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_INFO(this->get_logger(), "Service '/xarm/motion_enable' not available, waiting...");
    }

    // Send the request asynchronously and define a callback to handle the response
    auto future_result = motion_enable_client->async_send_request(request, [this](rclcpp::Client<xarm_msgs::srv::SetInt16ById>::SharedFuture future) {
        auto response = future.get();
        if (response->ret == 0) {
            RCLCPP_INFO(this->get_logger(), "Set motion enable completed!");
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to call service motion_enable");
        }
    });

    // Log that the request has been sent
    RCLCPP_INFO(get_logger(), "Sending the request for setting motion enable...");
}


void XarmController::grasp_position_cb(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    // Check whether the received grasp point is valid based on its Z-value
    // It must be at or below the approach point (with a small margin of 0.01)
    if (msg->point.z <= approach_point_cam(2) + 0.01)
    {
        point_received = true;

        // Store the received grasp point in camera frame as a homogeneous 4D vector
        grasp_point_cam << msg->point.x, msg->point.y, msg->point.z, 1.0;

        // Log the received position and its reference frame
        RCLCPP_INFO(this->get_logger(), 
                    "Received object position: [%.3f, %.3f, %.3f] in frame: %s",
                    msg->point.x, msg->point.y, msg->point.z, msg->header.frame_id.c_str());
    }
    else
    {
        // Log that the grasp point is not valid (too high)
        RCLCPP_INFO(this->get_logger(), "No valid Grasp Point");
    }
}


void XarmController::grasp_orientation_cb(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
    // Mark that a valid normal vector (orientation) has been received
    normal_received = true;

    // Log the received orientation vector and its reference frame
    RCLCPP_INFO(this->get_logger(), 
                "Received grasp orientation: [%.3f, %.3f, %.3f] in frame: %s",
                msg->vector.x, msg->vector.y, msg->vector.z, msg->header.frame_id.c_str());

    // Store the received normal vector in the camera coordinate frame
    normal_vector_cam = Eigen::Vector3d(msg->vector.x, msg->vector.y, msg->vector.z);
}


void XarmController::check_object_cb(const std_msgs::msg::Bool::SharedPtr msg)
{
    // Update internal flag based on message data (true = object detected, false = not detected)
    checked_object = msg->data;

    // Log the detection result
    RCLCPP_INFO(this->get_logger(), "Object %s", checked_object ? "Detected" : "Not detected");

    // Mark that object presence has been confirmed (or not) via message
    check_received = true;
}

void XarmController::set_robot_mode(int mode)
{
    // Prepare the service request with the desired mode
    auto request = std::make_shared<xarm_msgs::srv::SetInt16::Request>();
    request->data = mode;

    // Wait until the service is available
    while (!set_mode_client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    // Send the request asynchronously
    auto future_result = set_mode_client->async_send_request(
        request, 
        [this](rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedFuture future) 
        {
            // Retrieve and evaluate the response
            auto response = future.get();
            if (response->ret == 0) {
                RCLCPP_INFO(this->get_logger(), "Set robot mode completed!");
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to call service set_mode");
            }
        }
    );

    // Log confirmation of request sent
    RCLCPP_INFO(get_logger(), "SENDING THE REQUEST FOR SETTING MODE\n");
}


void XarmController::set_robot_state(int state)
{
    // Create a new request with the desired state (e.g. enable/disable, start/stop)
    auto request = std::make_shared<xarm_msgs::srv::SetInt16::Request>();
    request->data = state;

    // Wait for the service to be available before sending the request
    while (!set_state_client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    // Send the request asynchronously and handle the result via a lambda callback
    auto future_result = set_state_client->async_send_request(
        request,
        [this](rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedFuture future)
        {
            auto response = future.get();
            if (response->ret == 0) {
                RCLCPP_INFO(this->get_logger(), "Set robot state completed!");
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to call service set_state");
            }
        }
    );
     // Log confirmation of request sent
    RCLCPP_INFO(get_logger(), "SENDING THE REQUEST FOR SETTING STATE\n");
}


void XarmController::clean_robot_errors()
{
    // Create an empty request to call the clean error service
    auto request = std::make_shared<xarm_msgs::srv::Call::Request>();

    while (!clean_error_client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_INFO(this->get_logger(), "clean_error service not available, waiting again...");
    }

    // Send request asynchronously to clean error state
    auto future_result = clean_error_client->async_send_request(
        request,
        [this](rclcpp::Client<xarm_msgs::srv::Call>::SharedFuture future)
        {
            auto response = future.get();
            if (response->ret == 0) {
                RCLCPP_INFO(this->get_logger(), "Clean errors completed!");
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to call service clean_error");
            }
        }
    );
}


void XarmController::set_vacuum_gripper(bool on)
{
    // Create a shared pointer request object for the VacuumGripperCtrl service
    auto request = std::make_shared<xarm_msgs::srv::VacuumGripperCtrl::Request>();

    // Set the desired vacuum gripper state (on/off)
    request->on = on;

    // Additional parameters for the service call:
    // wait: whether to wait for the action to complete before returning (false = don't wait)
    request->wait = false;

    // timeout: how long to wait for the operation to succeed before giving up (in seconds)
    request->timeout = 3.0;

    // delay_sec: delay before starting the action, here set to zero (no delay)
    request->delay_sec = 0.0;

    // sync: whether the call should be synchronous or asynchronous (false = async)
    request->sync = false;

    // hardware_version: specifies the hardware version of the gripper, set to 1 here
    request->hardware_version = 1;

    // Wait for the vacuum gripper service to become available
    while (!set_vacuum_gripper_client->wait_for_service(std::chrono::seconds(1)))
    {
        // Inform the user that the service is still not available and we're waiting
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    // Send the request asynchronously to the vacuum gripper service
    // The lambda callback handles the response once it's received
    auto future_result = set_vacuum_gripper_client->async_send_request(
        request,
        [this](rclcpp::Client<xarm_msgs::srv::VacuumGripperCtrl>::SharedFuture future)
        {
            // Get the service response
            auto response = future.get();

            // Check if the service call succeeded (ret == 0 means success)
            if (response->ret == 0)
            {
                RCLCPP_INFO(this->get_logger(), "Set vacuum gripper completed!");
            }
            else
            {
                // Warn if the service call failed
                RCLCPP_WARN(this->get_logger(), "Failed to call service set_vacuum_gripper");
            }
        });

    // Log that the request to control the vacuum gripper has been sent
    RCLCPP_INFO(get_logger(), "SENDING THE REQUEST FOR SETTING VACUUM GRIPPER\n");
}


void XarmController::send_target_pose(const std::array<float,6>& target_pose, float speed, float acc)
{
    // Set the robot mode to 0, usually representing idle or default mode
    set_robot_mode(0);

    // Set the robot state to 0, commonly indicating ready or standby state
    set_robot_state(0);

    // Initialize a vector to store the pose (position and orientation)
    // The robot expects position values in millimeters, so convert from meters by multiplying by 1000
    std::vector<float> pose = {0, 0, 0, 0, 0, 0};

    // Create a request object for the MoveCartesian service, which controls the robot's end-effector pose
    auto request = std::make_shared<xarm_msgs::srv::MoveCartesian::Request>();

    // Fill the position part of the pose by converting from meters to millimeters (x, y, z)
    pose[0] = target_pose[0] * 1000;  // X position in mm
    pose[1] = target_pose[1] * 1000;  // Y position in mm
    pose[2] = target_pose[2] * 1000;  // Z position in mm

    // Copy the orientation part of the pose (roll, pitch, yaw) as expected by the robot, usually in radians or degrees
    pose[3] = target_pose[3];
    pose[4] = target_pose[4];
    pose[5] = target_pose[5];

    // Assign the prepared pose to the request message
    request->pose = pose;

    // Set the desired speed and acceleration for the motion command
    request->speed = speed;
    request->acc = acc;

    // Set movement time to zero, meaning the robot will move using speed and acceleration parameters without a fixed time
    request->mvtime = 0;

    // Wait until the move service becomes available, retrying every second if necessary
    while (!robot_move_client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    // Send the request asynchronously and define a callback to handle the response
    auto future_result = robot_move_client->async_send_request(
        request,
        [this](rclcpp::Client<xarm_msgs::srv::MoveCartesian>::SharedFuture future)
        {
            // Retrieve the response from the service call
            auto response = future.get();

            // Check if the service call was successful (ret == 0 means success)
            if (response->ret == 0)
            {
                RCLCPP_INFO(this->get_logger(), "Target pose sent successfully!");
                // Mark that the target pose command has been sent
                target_pose_sent = true;
            }
            else
            {
                // Warn if the service call failed
                RCLCPP_WARN(this->get_logger(), "Failed to call service to set position");
            }
        }
    );

    // Log that the request has been sent to move the robot
    RCLCPP_INFO(get_logger(), "Sending the move request\n");
}




void XarmController::robot_states_cb(const xarm_msgs::msg::RobotMsg::SharedPtr msg)
{
    // Log the current state of the robot received from the message
    RCLCPP_INFO(this->get_logger(), "Robot State: %d", msg->state);

    // Save the previous robot state before updating
    // Possible states: 0 = Ready (idle), 1 = Busy (moving), 2 = Idle, 4 = Error
    last_robot_state = robot_state;

    // Update the current robot state with the new value from the message
    robot_state = msg->state;

    // Update the robot's pose vector with the latest pose from the message
    robot_pose = msg->pose;

    // Convert each pose element from float to double for internal precision use
    for (int i = 0; i < 6; i++) {
        robot_pose[i] = static_cast<double>(msg->pose[i]);
    }
}


bool XarmController::check_robot_pose(const std::array<float,6>& robot_pose,
                                     const std::array<float,6>& target_pose,
                                     float pos_tolerance,
                                     float ang_tolerance)
{
    // Log the current robot pose values for debugging
    RCLCPP_INFO(this->get_logger(), "Robot Pose: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                robot_pose[0], robot_pose[1], robot_pose[2],
                robot_pose[3], robot_pose[4], robot_pose[5]);

    // Log the target pose values for comparison
    RCLCPP_INFO(this->get_logger(), "Target Pose: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                target_pose[0], target_pose[1], target_pose[2],
                target_pose[3], target_pose[4], target_pose[5]);

    // Check position difference between current robot pose and target pose
    // Note: target_pose is in meters, so convert to millimeters by multiplying by 1000
    for (int i = 0; i < 3; i++) {
        float diff_pos = std::fabs(robot_pose[i] - target_pose[i] * 1000.0);
        if (diff_pos > pos_tolerance) {
            // Log a warning if position difference exceeds the allowed tolerance
            RCLCPP_WARN(this->get_logger(), "Not in position tolerance (Diff = %.2f mm for index [%d])", diff_pos, i);
            return false;
        }
    }

    // Check orientation difference between current and target pose
    // Convert roll-pitch-yaw angles to quaternion representations for comparison
    tf2::Quaternion q_robot;
    q_robot.setRPY(robot_pose[3], robot_pose[4], robot_pose[5]);

    tf2::Quaternion q_target;
    q_target.setRPY(target_pose[3], target_pose[4], target_pose[5]);

    // Calculate the shortest angular difference between the two quaternions
    double angle_diff = q_robot.angleShortestPath(q_target);

    // Compare the angle difference with the angular tolerance
    if (angle_diff > ang_tolerance) {
        // Log a warning if angular difference is larger than allowed tolerance
        RCLCPP_WARN(this->get_logger(), "Not in angle tolerance (Diff = %.2f rad)", angle_diff);
        return false;
    }

    // If both position and orientation are within tolerances, return true
    return true;
}




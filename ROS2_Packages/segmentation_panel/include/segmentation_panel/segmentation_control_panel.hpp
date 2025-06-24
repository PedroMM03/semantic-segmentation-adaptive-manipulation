/*
 * Project name: Semantic Segmentation for Adaptive Manipulation with Industrial Robots
 * Author: Pedro Martin
 * Electronics, Robotics and Mechatronics Engineering - University of Malaga
 * Date: 2025
 */

#ifndef SEGMENTATION_CONTROL_PANEL_HPP
#define SEGMENTATION_CONTROL_PANEL_HPP

// Include base RViz panel class
#include <rviz_common/panel.hpp>
// Include ROS2 client library
#include <rclcpp/rclcpp.hpp>
// Include standard ROS message types
#include <std_msgs/msg/string.hpp>
#include "nlohmann/json.hpp"  // JSON library for parsing segmented object data
#include "xarm6_controller/srv/process_object.hpp"  // Custom service for processing objects
#include "std_msgs/msg/int16.hpp"  // Integer message used for process step communication

// Qt GUI includes for building the custom panel interface
#include <QLabel>
#include <QComboBox>
#include <QVBoxLayout>
#include <QTableWidget>
#include <QPushButton>
#include <QHeaderView>
#include <QWidget>
#include <QStringList>
#include <iostream>
#include <thread>
#include <memory>

using json = nlohmann::json;

namespace segmentation_panel
{

// Custom RViz panel class to display and interact with segmented objects
class SegmentationControlPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  // Constructor and destructor
  SegmentationControlPanel(QWidget * parent = nullptr);
  ~SegmentationControlPanel() override;

private:
  // Callback to receive segmented objects data as a JSON string message
  void segmented_objects_json_cb(const std_msgs::msg::String::SharedPtr msg);

  // Callback to receive process step updates (integer) from another node
  void process_step_cb(const std_msgs::msg::Int16::SharedPtr msg);

  // Update the Qt table widget with the latest segmented objects parsed from JSON
  void updateTable(const nlohmann::json& json_array);

  // Structure to hold information about the currently selected object in the panel
  struct SelectedObject {
    int id = -1;               // Unique identifier of the object
    std::string class_name;    // Object classification label
    float x = 0.0, y = 0.0, z = 0.0;  // Object position coordinates
    float dist;                // Distance metric (e.g. from sensor or robot)
    bool valid = false;        // Flag indicating if the selected object data is valid
  };

  bool accept;  // Flag to indicate if the current selection was accepted

  SelectedObject selected_register;  // Stores the currently selected object information

  int mode;       // Mode selector (0=Instance, 1=Class, 2=Classification), controls filtering/display logic
  bool processing;  // Indicates if the panel is currently processing a request

  // Qt widgets that make up the control panel UI
  QLabel * label;           // Descriptive text label
  QComboBox * combo_mode;   // Dropdown menu for selecting mode/filter option
  QTableWidget * table;     // Table to display segmented objects and their attributes
  QPushButton * accept_button;  // Button to confirm selection or trigger processing
  QLabel * status_label;    // Label showing current status or messages

  // ROS node handle for communication within the panel
  rclcpp::Node::SharedPtr node;

  // Publisher to send the selected object information to other nodes
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr selected_object_pub;

  // Subscriptions to receive segmented objects and process step updates
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr segmented_objects_json_sub;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr process_step_sub;

  // Client for the service that triggers processing of selected objects on the robot/controller
  rclcpp::Client<xarm6_controller::srv::ProcessObject>::SharedPtr process_object_client;

  // Thread dedicated to spinning the ROS node to process callbacks asynchronously
  std::shared_ptr<std::thread> rclcpp_thread;

  // Stores the last received segmented objects data as JSON for access and updates
  nlohmann::json last_segmented_objs_data;
};

}  // namespace segmentation_panel

#endif  // SEGMENTATION_CONTROL_PANEL_HPP

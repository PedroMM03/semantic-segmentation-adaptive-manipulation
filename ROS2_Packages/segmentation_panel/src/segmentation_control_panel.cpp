/*
 * Project name: Semantic Segmentation for Adaptive Manipulation with Industrial Robots
 * Author: Pedro Martin
 * Electronics, Robotics and Mechatronics Engineering - University of Malaga
 * Date: 2025
 */



#include "segmentation_panel/segmentation_control_panel.hpp"
#include <pluginlib/class_list_macros.hpp>

using std::placeholders::_1;
using nlohmann::json;

namespace segmentation_panel
{

SegmentationControlPanel::SegmentationControlPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
    // Initialize ROS2 node for this panel
    node = std::make_shared<rclcpp::Node>("segmentation_control_panel_node");

    // QoS profile with reliable and transient_local settings
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliable();
    qos.transient_local();

    // Publisher to send the selected object as JSON string
    selected_object_pub = node->create_publisher<std_msgs::msg::String>("/segmentation_panel/selected_object", qos);

    // Subscriber for segmented objects JSON data
    segmented_objects_json_sub = node->create_subscription<std_msgs::msg::String>(
        "/zed_vision/segmented_objects", qos,
        std::bind(&SegmentationControlPanel::segmented_objects_json_cb, this, _1));

    // Subscriber for xarm6 process step updates
    process_step_sub = node->create_subscription<std_msgs::msg::Int16>(
        "/xarm6_controller/process_step", qos,
        std::bind(&SegmentationControlPanel::process_step_cb, this, _1));

    // Service client to request processing of selected objects
    process_object_client = node->create_client<xarm6_controller::srv::ProcessObject>("process_object_service");

    // Spin the node in a separate thread to keep callbacks alive
    rclcpp_thread = std::make_shared<std::thread>([this]() {
        rclcpp::spin(node);
    });

    // Initialize state variables
    processing = false;
    accept = false;

    // Setup Qt GUI elements and layout
    auto * layout = new QVBoxLayout;

    label = new QLabel("Semantic Segmentation-Based Manipulation Panel");
    layout->addWidget(label);

    combo_mode = new QComboBox();
    combo_mode->addItems(QStringList() << "Instance" << "Class" << "Classification");
    layout->addWidget(new QLabel("Operation Mode:"));
    layout->addWidget(combo_mode);

    // Store current mode index (0=Instance, 1=Class, 2=Classification)
    mode = combo_mode->currentIndex();

    // Connect mode change signal to update mode variable and log
    connect(combo_mode, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int index) {
                mode = index;
                std::cout << "Mode changed to: " << mode << std::endl;
            });

    // Label to show current process status
    status_label = new QLabel("Process status: Waiting...");
    layout->addWidget(status_label);

    // Table widget to show segmented objects list
    table = new QTableWidget();
    table->setColumnCount(3);
    table->setHorizontalHeaderLabels(QStringList() << "ID" << "CLASS" << "DISTANCE (m)");
    table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

    layout->addWidget(new QLabel("Segmented Objects:"));
    layout->addWidget(table);

    table->setSelectionBehavior(QAbstractItemView::SelectRows);
    table->setSelectionMode(QAbstractItemView::SingleSelection);

    // Connect table cell click to update selected object info
    connect(table, &QTableWidget::cellClicked, this, [this](int row, int /*column*/) {
        if (row < 0 || row >= static_cast<int>(last_segmented_objs_data.size())) {
            selected_register.valid = false;
            return;
        }

        // Update selected_register with data from clicked object
        const auto & obj = last_segmented_objs_data[row];
        selected_register.id = obj["id"].get<int>();
        selected_register.class_name = obj["class"].get<std::string>();
        selected_register.x = obj["x"].get<float>();
        selected_register.y = obj["y"].get<float>();
        selected_register.z = obj["z"].get<float>();
        selected_register.dist = obj["dist"].get<float>();
        selected_register.valid = true;

        std::cout << "Selected object: ID=" << selected_register.id
                  << ", CLASS=" << selected_register.class_name << std::endl;
    });

    // Accept button to confirm selection and trigger processing
    accept_button = new QPushButton("Accept");
    layout->addWidget(accept_button);

    // Connect accept button click to send request and publish selected object
    connect(accept_button, &QPushButton::clicked, this, [this]() {
        accept = true;

        // For Instance or Class modes, require a valid selected object
        if (mode == 0 || mode == 1) {
            if (!selected_register.valid) {
                std::cerr << "No object selected!" << std::endl;
                return;
            }
        }

        // Avoid new requests if a process is already ongoing
        if (!processing) {
            if (mode == 2) { // Classification mode picks first object automatically
                if (last_segmented_objs_data.empty()) {
                    std::cerr << "No segmented objects available!" << std::endl;
                    return;
                }
                const auto & obj = last_segmented_objs_data[0];
                selected_register.id = obj["id"].get<int>();
                selected_register.class_name = obj["class"].get<std::string>();
                selected_register.x = obj["x"].get<float>();
                selected_register.y = obj["y"].get<float>();
                selected_register.z = obj["z"].get<float>();
                selected_register.dist = obj["dist"].get<float>();
                selected_register.valid = true;
            }

            // Prepare service request with selected object data
            auto request = std::make_shared<xarm6_controller::srv::ProcessObject::Request>();
            request->id = selected_register.id;
            request->class_name = selected_register.class_name;
            request->aprox_x = selected_register.x;
            request->aprox_y = selected_register.y;
            request->aprox_z = selected_register.z;

            // Publish selected object JSON message
            json selected_obj_json = {
                {"id", selected_register.id},
                {"class", selected_register.class_name}
            };
            std_msgs::msg::String msg;
            msg.data = selected_obj_json.dump();
            selected_object_pub->publish(msg);

            // Call processing service asynchronously and print result
            process_object_client->async_send_request(request,
                [this](rclcpp::Client<xarm6_controller::srv::ProcessObject>::SharedFuture future) {
                    auto response = future.get();
                    if (response->success) {
                        std::cout << "Service call succeeded: " << response->message << std::endl;
                    } else {
                        std::cerr << "Service call failed: " << response->message << std::endl;
                    }
                });

        } else {
            std::cout << "Object is being processed " << std::endl;
        }
    });

    // Set the panel layout
    setLayout(layout);
}

SegmentationControlPanel::~SegmentationControlPanel()
{
    // Properly shutdown ROS2 and join the thread on destruction
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    if (rclcpp_thread && rclcpp_thread->joinable()) {
        rclcpp_thread->join();
    }
}

// Callback for receiving process step updates from xarm6 controller
void SegmentationControlPanel::process_step_cb(const std_msgs::msg::Int16::SharedPtr msg)
{
    int step = msg->data;

    // Handle logic depending on current mode and process step
    switch (mode) {
        case 0: // Instance mode
            processing = (step != 1); // Processing if not waiting step
            accept = false;
            break;

        case 1: // Class mode
            if (step == 1) {
                // Look for next object of the same class to process
                bool continue_processing = false;
                for (const auto & obj : last_segmented_objs_data) {
                    if (obj["class"] == selected_register.class_name &&
                        obj["id"].get<int>() != selected_register.id) {

                        // Update selected object to next matching one
                        selected_register.id = obj["id"].get<int>();
                        selected_register.class_name = obj["class"].get<std::string>();
                        selected_register.x = obj["x"].get<float>();
                        selected_register.y = obj["y"].get<float>();
                        selected_register.z = obj["z"].get<float>();
                        selected_register.dist = obj["dist"].get<float>();
                        selected_register.valid = true;

                        // Prepare and publish new service request and JSON
                        auto request = std::make_shared<xarm6_controller::srv::ProcessObject::Request>();
                        request->id = selected_register.id;
                        request->class_name = selected_register.class_name;
                        request->aprox_x = selected_register.x;
                        request->aprox_y = selected_register.y;
                        request->aprox_z = selected_register.z;

                        std_msgs::msg::String msg;
                        json selected_obj_json = {
                            {"id", selected_register.id},
                            {"class", selected_register.class_name}
                        };
                        msg.data = selected_obj_json.dump();
                        selected_object_pub->publish(msg);

                        // Call service asynchronously and log results
                        process_object_client->async_send_request(request,
                            [this](rclcpp::Client<xarm6_controller::srv::ProcessObject>::SharedFuture future) {
                                auto response = future.get();
                                if (response->success) {
                                    std::cout << "Service call succeeded: " << response->message << std::endl;
                                } else {
                                    std::cerr << "Service call failed: " << response->message << std::endl;
                                }
                            });

                        continue_processing = true;
                        break;
                    }
                }
                processing = continue_processing;
            }
            accept = false;
            break;

        case 2: // Classification mode
            if (step == 1) {
                if (accept == true) {
                    if (last_segmented_objs_data.empty()) {
                        std::cerr << "No segmented objects available" << std::endl;
                        processing = false;
                        return;
                    }
                    processing = true;
                    // Select the first segmented object
                    const auto & obj = last_segmented_objs_data[0];
                    selected_register.id = obj["id"].get<int>();
                    selected_register.class_name = obj["class"].get<std::string>();
                    selected_register.x = obj["x"].get<float>();
                    selected_register.y = obj["y"].get<float>();
                    selected_register.z = obj["z"].get<float>();
                    selected_register.dist = obj["dist"].get<float>();
                    selected_register.valid = true;

                    // Prepare request and publish selected object JSON
                    auto request = std::make_shared<xarm6_controller::srv::ProcessObject::Request>();
                    request->id = selected_register.id;
                    request->class_name = selected_register.class_name;
                    request->aprox_x = selected_register.x;
                    request->aprox_y = selected_register.y;
                    request->aprox_z = selected_register.z;

                    json selected_obj_json = {
                        {"id", selected_register.id},
                        {"class", selected_register.class_name}
                    };

                    std_msgs::msg::String msg;
                    msg.data = selected_obj_json.dump();
                    selected_object_pub->publish(msg);

                    // Call processing service asynchronously
                    process_object_client->async_send_request(request,
                        [this](rclcpp::Client<xarm6_controller::srv::ProcessObject>::SharedFuture future) {
                            auto response = future.get();
                            if (response->success) {
                                std::cout << "Service call succeeded: " << response->message << std::endl;
                            } else {
                                std::cerr << "Service call failed: " << response->message << std::endl;
                            }
                        });
                }
            }
            break;
    }

    // Map process step integer to status string for GUI display
    QString status_text;
    switch (step) {
        case 0: status_text = "Initialization"; break;
        case 1: status_text = "Waiting..."; break;
        case 2: status_text = "Moving to approach point"; break;
        case 3: status_text = "Getting grasp point and orientation"; break;
        case 4: status_text = "Above object (not oriented)"; break;
        case 5: status_text = "Above object (oriented)"; break;
        case 6: status_text = "Picking object"; break;
        case 7: status_text = "Lifting object"; break;
        case 8: status_text = "Moving to verification point"; break;
        case 9: status_text = "Verifying grasp"; break;
        case 10: status_text = "Moving above destination"; break;
        case 11: status_text = "Placing object at destination"; break;
        case 12: status_text = "Object placed, returning to home"; break;
    }

    // Update the status label in the GUI thread safely
    QMetaObject::invokeMethod(this, [this, status_text]() {
        status_label->setText("Process status: " + status_text);
    });
}

// Callback for receiving segmented objects JSON messages
void SegmentationControlPanel::segmented_objects_json_cb(const std_msgs::msg::String::SharedPtr msg)
{
    try {
        // Parse JSON array of segmented objects
        json parsed = json::parse(msg->data);
        last_segmented_objs_data = parsed;

        // Update table in GUI thread
        QMetaObject::invokeMethod(this, [this, parsed]() {
            updateTable(parsed);
        });

    } catch (const std::exception & e) {
        std::cerr << "Failed to parse JSON: " << e.what() << std::endl;
    }
}

// Update the QTableWidget with the current segmented objects data
void SegmentationControlPanel::updateTable(const json& json_array)
{
    table->clearContents();
    table->setRowCount(json_array.size());

    for (size_t i = 0; i < json_array.size(); ++i) {
        QString id_str = QString::number(json_array[i]["id"].get<int>());
        QString class_str = QString::fromStdString(json_array[i]["class"].get<std::string>());
        QString dist_str = QString::number(json_array[i]["dist"].get<float>(), 'f', 2);

        table->setItem(i, 0, new QTableWidgetItem(id_str));
        table->setItem(i, 1, new QTableWidgetItem(class_str));
        table->setItem(i, 2, new QTableWidgetItem(dist_str));
    }
}

}  // namespace segmentation_panel

// Export this panel class as a plugin for RViz2
PLUGINLIB_EXPORT_CLASS(segmentation_panel::SegmentationControlPanel, rviz_common::Panel)

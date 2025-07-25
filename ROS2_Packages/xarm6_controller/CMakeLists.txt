cmake_minimum_required(VERSION 3.5)
project(xarm6_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ProcessObject.srv"
  DEPENDENCIES builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)


include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIRS})



#include_directories(/opt/ros/foxy/include/tf2_eigen)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


add_executable(xarm6_controller_node
src/xarm6_controller.cpp
src/main.cpp
)




ament_target_dependencies(xarm6_controller_node
rclcpp
std_msgs
geometry_msgs
xarm_msgs
tf2
tf2_ros
tf2_eigen
tf2_geometry_msgs
Eigen3
)


rosidl_target_interfaces(xarm6_controller_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")


install(TARGETS
xarm6_controller_node
DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()

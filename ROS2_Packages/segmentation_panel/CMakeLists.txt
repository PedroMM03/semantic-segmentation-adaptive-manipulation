cmake_minimum_required(VERSION 3.5)
project(segmentation_panel)

# === Configuración del estándar de C++
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === Dependencias requeridas ===
find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(xarm6_controller REQUIRED)


# === Configuración para Qt y moc ===
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

include_directories(include)

# === MOC: procesar headers con Q_OBJECT ===
set(MOC_FILES
  include/segmentation_panel/segmentation_control_panel.hpp
)
qt5_wrap_cpp(MOC_SOURCES ${MOC_FILES})

# === Crear biblioteca compartida para el plugin ===
add_library(${PROJECT_NAME} SHARED
  src/segmentation_control_panel.cpp
  ${MOC_SOURCES}
)

ament_target_dependencies(${PROJECT_NAME}
  rviz_common
  pluginlib
  rclcpp
  std_msgs
  xarm6_controller
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
)

# === Instalación ===
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES
  resource/segmentation_panel_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# === Exportar el plugin a RViz ===
pluginlib_export_plugin_description_file(rviz_common resource/segmentation_panel_plugin.xml)

ament_package()

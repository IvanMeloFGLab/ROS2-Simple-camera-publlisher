cmake_minimum_required(VERSION 3.10)
project(cpp_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Asegurar compatibilidad gnu++2a (C++20), Jazzy soporta C++17+ y C++20.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

include_directories(include ${Qt5Widgets_INCLUDE_DIRS})

set(camera_ui_node_SRCS
  src/camera_ui.cpp
  src/camera_ui_window.cpp
)
set(camera_ui_node_HDRS
  include/cpp_camera/camera_ui_window.h
)

qt5_wrap_cpp(camera_ui_node_MOCS ${puzzlesim_node_HDRS})

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

add_executable(camera_pub src/camera_publisher_c.cpp)
ament_target_dependencies(camera_pub rclcpp sensor_msgs cv_bridge OpenCV)

add_executable(camera_sub_com src/camera_subs_comp.cpp)
ament_target_dependencies(camera_sub_com rclcpp sensor_msgs OpenCV)

add_executable(camera_ui ${camera_ui_node_SRCS} ${camera_ui_node_MOCS})
target_link_libraries(camera_ui Qt5::Widgets)
ament_target_dependencies(camera_ui rclcpp ament_index_cpp)
target_link_libraries(camera_ui "${cpp_typesupport_target}")

install(TARGETS
  camera_pub
  camera_sub_com
  camera_ui
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

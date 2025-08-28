cmake_minimum_required(VERSION 3.10.1)
project(detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Release)

# todo:如果部署成功要将该功能包安装到environment中
set(ncnn_DIR "/home/ww/ncnn/install/lib/cmake/ncnn")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(image_task_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(interface REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ncnn REQUIRED)
find_package(OpenMP REQUIRED)
if(OpenMP_FOUND)
    message("OpenMP found")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

include_directories(
    include/
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
)

add_library(
    dashboard_detector SHARED plugins/dashboard_detector.cpp plugins/nanodet.cpp plugins/yolov8_pose.cpp
)
add_executable(
    detect_node
    src/detect_node.cpp
)

ament_target_dependencies(dashboard_detector
    rclcpp sensor_msgs interface rclcpp_lifecycle image_task_core pluginlib
)
ament_target_dependencies(detect_node
    rclcpp sensor_msgs interface rclcpp_lifecycle image_task_core pluginlib
)

target_link_libraries(
    dashboard_detector
    ${OpenCV_LIBS}
    ncnn
)
target_link_libraries(
    detect_node
    ${OpenCV_LIBS}
    ncnn
)

install(DIRECTORY include
  DESTINATION include
)
install(TARGETS detect_node
    DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS dashboard_detector
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY config weights
    DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(image_task_core detector.xml)

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

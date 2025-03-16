cmake_minimum_required(VERSION 3.8)
project(mine_detector)

cmake_policy(SET CMP0057 NEW)

set(CMAKE_CXX_STANDARD 11)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
#set(YAML_CPP_DIR "/usr/local/lib/cmake/yaml-cpp")
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(message_filters REQUIRED)
if(yaml-cpp_FOUND)
  message(STATUS "Found yaml-cpp: ${yaml-cpp_VERSION}")
  include_directories(${yaml-cpp_INCLUDE_DIRS})
endif()
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(msg_interfaces REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
        DIRECTORY src
)

ament_target_dependencies(
        ${PROJECT_NAME}
        OpenCV
        cv_bridge
        rclcpp
        sensor_msgs
        rclcpp_components
        geometry_msgs
        tf2
)

target_include_directories(${PROJECT_NAME} PUBLIC
        ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
)
target_link_libraries(${PROJECT_NAME}
        yaml-cpp
)
target_include_directories(mine_detector PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

RCLCPP_COMPONENTS_REGISTER_NODE(${PROJECT_NAME}
        PLUGIN mine_detector
        EXECUTABLE mine_detector_node
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
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

ament_auto_package()

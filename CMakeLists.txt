cmake_minimum_required(VERSION 3.20)
project(rc_car_interface VERSION 0.3.0 LANGUAGES C CXX)
option(RC_CAR_STANDALONE "Build the Linux transport and tests without ROS" OFF)
find_package(Threads REQUIRED)

add_library(rc_car_link STATIC src/serial.cpp)
target_compile_features(rc_car_link PUBLIC cxx_std_17)
target_include_directories(rc_car_link PUBLIC include)
target_compile_options(rc_car_link PRIVATE -Wall -Wextra -Werror)
add_executable(rc_car_maintenance_link src/maintenance_link.cpp)
target_compile_options(rc_car_maintenance_link PRIVATE -Wall -Wextra -Werror)
target_link_libraries(rc_car_maintenance_link PRIVATE rc_car_link Threads::Threads)
add_executable(rc_car_ctl src/ctl.cpp)
target_compile_options(rc_car_ctl PRIVATE -Wall -Wextra -Werror)
target_link_libraries(rc_car_ctl PRIVATE rc_car_link Threads::Threads)

if(RC_CAR_STANDALONE OR BUILD_TESTING)
  enable_testing()
  add_executable(rc_car_link_test tests/link_test.cpp)
  target_compile_options(rc_car_link_test PRIVATE -Wall -Wextra -Werror)
  target_link_libraries(rc_car_link_test PRIVATE rc_car_link util)
  add_test(NAME rc_car_link_test COMMAND rc_car_link_test "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures")
endif()

if(RC_CAR_STANDALONE)
  return()
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DriveCommand.msg"
  "msg/PanCommand.msg"
  "msg/VehicleStatus.msg"
  "srv/SetMode.srv"
  "srv/SetRuntimeMode.srv"
  DEPENDENCIES builtin_interfaces
)
rosidl_get_typesupport_target(cpp_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
add_executable(vehicle_interface src/node.cpp)
target_compile_options(vehicle_interface PRIVATE -Wall -Wextra -Werror)
target_link_libraries(vehicle_interface rc_car_link Threads::Threads "${cpp_typesupport}")
ament_target_dependencies(vehicle_interface rclcpp rcl_interfaces diagnostic_msgs builtin_interfaces)

if(BUILD_TESTING)
  add_executable(rc_car_ros_test tests/ros_test.cpp)
  target_compile_options(rc_car_ros_test PRIVATE -Wall -Wextra -Werror)
  target_link_libraries(rc_car_ros_test rc_car_link Threads::Threads util "${cpp_typesupport}")
  ament_target_dependencies(rc_car_ros_test rclcpp rcl_interfaces diagnostic_msgs builtin_interfaces)
  add_test(NAME rc_car_ros_test COMMAND rc_car_ros_test)
  set_tests_properties(rc_car_ros_test PROPERTIES TIMEOUT 20 ENVIRONMENT
    "ROS_DOMAIN_ID=161;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST;LD_LIBRARY_PATH=${CMAKE_CURRENT_BINARY_DIR}:$ENV{LD_LIBRARY_PATH}")
endif()

install(TARGETS vehicle_interface rc_car_maintenance_link rc_car_ctl DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()

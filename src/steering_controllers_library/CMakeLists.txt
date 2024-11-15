cmake_minimum_required(VERSION 3.16)
project(steering_controllers_library LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion)
endif()

# Find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
  tf2
  tf2_msgs
  tf2_geometry_msgs
  ackermann_msgs
  # amt212ev_interfaces
)

include_directories(
  ${amt212ev_interfaces_INCLUDE_DIRS}
)

find_package(amt212ev_interfaces REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Generate parameter library
generate_parameter_library(steering_controllers_library_parameters
  src/steering_controllers_library.yaml
)

# Add the shared library
# Add the shared library
add_library(
  steering_controllers_library
  SHARED
  src/steering_controllers_library.cpp
  src/steering_odometry.cpp  # Re-add this line
)

target_compile_features(steering_controllers_library PUBLIC cxx_std_17)
target_include_directories(steering_controllers_library PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/steering_controllers_library>"
)
target_link_libraries(steering_controllers_library
  steering_controllers_library_parameters
)
ament_target_dependencies(
  steering_controllers_library
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Define compile definitions
target_compile_definitions(steering_controllers_library PRIVATE
  "STEERING_CONTROLLERS_BUILDING_DLL"
  "_USE_MATH_DEFINES"
)

# Add the executable node for steering_odometry.cpp
add_executable(odom_node src/odom_node.cpp)
target_compile_features(odom_node PUBLIC cxx_std_17)
target_include_directories(odom_node PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/steering_controllers_library>"
)
target_link_libraries(odom_node
  steering_controllers_library  # Link against the library
)
ament_target_dependencies(
  odom_node
  amt212ev_interfaces
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Install the executable
install(TARGETS odom_node
  DESTINATION lib/${PROJECT_NAME}
)
# If building tests
if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  # Add the test executable using ament_add_gmock
  ament_add_gmock(
    steering_controllers_library_test
    test/test_steering_controllers_library.cpp
  )
  target_include_directories(steering_controllers_library_test PRIVATE include)
  target_link_libraries(steering_controllers_library_test
    steering_controllers_library
  )
  ament_target_dependencies(
    steering_controllers_library_test
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
  )

  # Add GMock test for steering odometry
  ament_add_gmock(
    test_steering_odometry
    test/test_steering_odometry.cpp
  )
  target_link_libraries(test_steering_odometry
    steering_controllers_library
  )
  ament_target_dependencies(
    test_steering_odometry
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
  )
endif()

# Install targets
install(TARGETS
  steering_controllers_library
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/steering_controllers_library
)

install(
  TARGETS steering_controllers_library steering_controllers_library_parameters
  EXPORT export_steering_controllers_library
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_steering_controllers_library HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()

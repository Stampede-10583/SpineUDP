cmake_minimum_required(VERSION 3.20)
project(spine_link LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(spine_interfaces REQUIRED)

# ROS-free core: wire format, Stream 3 read rule, order gate.
add_library(spine_link_core STATIC src/trigger_table.cpp)
target_include_directories(spine_link_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
set_target_properties(spine_link_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Companion node (component + standalone executable "companion_node").
add_library(spine_companion SHARED src/companion_node.cpp)
target_link_libraries(spine_companion PUBLIC spine_link_core
  rclcpp::rclcpp rclcpp_components::component ${diagnostic_msgs_TARGETS}
  ${spine_interfaces_TARGETS})
rclcpp_components_register_node(spine_companion
  PLUGIN "spine::SpineCompanionNode" EXECUTABLE companion_node)

# Synthetic pose node (component + standalone executable "synthetic_pose_node").
add_library(spine_synthetic SHARED src/synthetic_pose_node.cpp)
target_link_libraries(spine_synthetic PUBLIC spine_link_core
  rclcpp::rclcpp rclcpp_components::component ${spine_interfaces_TARGETS})
rclcpp_components_register_node(spine_synthetic
  PLUGIN "spine::SyntheticPoseNode" EXECUTABLE synthetic_pose_node)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS spine_companion spine_synthetic
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS spine_link_core EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
# Shared with the Java tests and the fake-RIO integration test.
install(FILES test/data/spine-link-vectors.txt DESTINATION share/${PROJECT_NAME}/test_data)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  set(SPINE_VECTORS_FILE "${CMAKE_CURRENT_SOURCE_DIR}/test/data/spine-link-vectors.txt")

  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol spine_link_core)
  target_compile_definitions(test_protocol PRIVATE SPINE_VECTORS_FILE="${SPINE_VECTORS_FILE}")

  ament_add_gtest(test_trigger_table test/test_trigger_table.cpp)
  target_link_libraries(test_trigger_table spine_link_core)

  ament_add_gtest(test_order_gate test/test_order_gate.cpp)
  target_link_libraries(test_order_gate spine_link_core)

  ament_add_gtest(test_pose_pipeline test/test_pose_pipeline.cpp)
  target_link_libraries(test_pose_pipeline spine_link_core)

  ament_add_gtest(test_odometry_tracker test/test_odometry_tracker.cpp)
  target_link_libraries(test_odometry_tracker spine_link_core)

  ament_add_gtest(test_se2 test/test_se2.cpp)
  target_link_libraries(test_se2 spine_link_core)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_package()

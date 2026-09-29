cmake_minimum_required(VERSION 3.20)
project(spine_bringup NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  # Fake RIO <-> companion + synthetic pose node over loopback UDP.
  add_launch_test(test/test_bench_integration.py TIMEOUT 120)
endif()

ament_package()

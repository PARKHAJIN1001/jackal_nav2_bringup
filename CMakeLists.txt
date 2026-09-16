cmake_minimum_required(VERSION 3.10)
project(jackal_nav2_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(tf_authority_probe src/tf_authority_probe.cpp)
ament_target_dependencies(tf_authority_probe rclcpp tf2_msgs)
target_link_libraries(tf_authority_probe nlohmann_json::nlohmann_json)

add_executable(static_costmap_node src/static_costmap_node.cpp)
ament_target_dependencies(static_costmap_node nav2_costmap_2d rclcpp)

add_executable(pointcloud_relay_node src/pointcloud_relay_node.cpp)
target_include_directories(pointcloud_relay_node PRIVATE include)
ament_target_dependencies(pointcloud_relay_node rclcpp sensor_msgs diagnostic_msgs)

install(
  TARGETS
    tf_authority_probe
    pointcloud_relay_node
    static_costmap_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/tf_localization_audit.py
    scripts/record_navigation_validation.py
    scripts/fast_livo_odom_adapter.py
    scripts/map_patch_node.py
    scripts/nav2_twist_stamper.py
    scripts/nav2_safety_guard.py
    scripts/nav2_safety_core.py
    scripts/amcl_recovery_monitor.py
    scripts/amcl_quality_monitor.py
    scripts/prepare_perception_config.py
    scripts/runtime_resource_audit.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
    docs
    maps
    rviz
    tools
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_relay_health test/test_relay_health.cpp)
  target_include_directories(test_relay_health PRIVATE include)
  ament_add_pytest_test(test_followup test/test_followup.py)

  ament_add_pytest_test(test_tf_localization_audit
    test/test_tf_localization_audit.py
  )

  ament_add_pytest_test(test_fast_livo_odom_adapter
    test/test_fast_livo_odom_adapter.py
  )
  ament_add_pytest_test(test_config_and_launch
    test/test_config_and_launch.py
  )
  ament_add_pytest_test(test_map_patch
    test/test_map_patch.py
  )
  ament_add_pytest_test(test_nav2_twist_stamper
    test/test_nav2_twist_stamper.py
  )
  ament_add_pytest_test(test_nav2_safety test/test_nav2_safety.py)

  # Python source files rely on the package LICENSE file.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

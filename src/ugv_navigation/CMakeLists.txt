cmake_minimum_required(VERSION 3.8)
project(ugv_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Dev 4: Nav2 planning & control configuration, launch, behavior trees, testbench
ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch behavior_trees
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(
  PROGRAMS scripts/nav_goal_testbench
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

  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_ros REQUIRED)

  # Static Dev 4 contract checks (no ROS graph needed)
  ament_add_pytest_test(test_dev4_config test/test_dev4_config.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 60
  )
  ament_add_pytest_test(test_testbench_core test/test_testbench_core.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 60
  )
  # Live graph check: launches navigation.launch.py, configures the Nav2 servers
  # (no activation, no TF/costmap data needed) and inspects topics/actions.
  ament_add_ros_isolated_pytest_test(test_navigation_graph test/test_navigation_graph.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 120
  )
endif()

ament_package()

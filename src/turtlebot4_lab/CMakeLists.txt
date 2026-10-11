cmake_minimum_required(VERSION 3.8)
project(turtlebot4_lab)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config maps DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/check_runtime.py scripts/save_map.py scripts/record_goals.py
  scripts/motion_probe.py scripts/validate_map.py scripts/cleanup_simulation.py scripts/teleop_hold.py DESTINATION lib/${PROJECT_NAME})
ament_package()

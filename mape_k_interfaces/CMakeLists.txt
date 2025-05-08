cmake_minimum_required(VERSION 3.5)
project(mape_k_interfaces)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CheckAnomaly.srv"
  DEPENDENCIES std_msgs
)

# Required to use the generated interfaces
ament_export_dependencies(rosidl_default_runtime)

# Install launch files (optional, if they exist)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

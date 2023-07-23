cmake_minimum_required(VERSION 3.8)
project(rwa5)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ariac_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

# directories
include_directories(include)

# service client executables
add_executable(service_client_exe src/service_client.cpp)
ament_target_dependencies(service_client_exe rclcpp std_srvs ariac_msgs)

# Client
add_executable(ship_order_exe src/shipOrder.cpp)
ament_target_dependencies(ship_order_exe rclcpp std_srvs std_msgs ariac_msgs)


# Create a variable for all dependencies
set(LOCATE_PARTS_TRAYS_DEPENDS
  rclcpp 
  std_srvs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  orocos_kdl
  ariac_msgs
)


# Find all dependencies
foreach(dependency IN ITEMS ${LOCATE_PARTS_TRAYS_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()


# Part and Tray
add_executable(locate_parts_trays_exe src/locate_parts_trays.cpp src/ariac_tf_util.cpp)
ament_target_dependencies(locate_parts_trays_exe ${LOCATE_PARTS_TRAYS_DEPENDS})


# install targets
install(TARGETS
  service_client_exe 
  ship_order_exe
  locate_parts_trays_exe
  DESTINATION lib/${PROJECT_NAME}
)

# install directories
install(DIRECTORY 
  include  
  launch
  config
  DESTINATION share/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME}
SCRIPTS_DESTINATION lib∕${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
	nodes/order_manager.py 
  nodes/submit_order.py
  nodes/end_comp_client_exe.py
	DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(rwa4)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ariac_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# directories
include_directories(include)


# service client executables
add_executable(service_client_exe src/service_client.cpp)
ament_target_dependencies(service_client_exe rclcpp std_srvs ariac_msgs)

# Client
add_executable(ship_order_exe src/shipOrder.cpp)
ament_target_dependencies(ship_order_exe rclcpp std_srvs std_msgs ariac_msgs)

# install targets
install(TARGETS
  service_client_exe 
  ship_order_exe
  DESTINATION lib/${PROJECT_NAME}
)

# install directories
install(DIRECTORY 
  include  
  launch
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

ament_package()
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

# # add cpp executable here
# add_executable()
# ament_target_dependencies( rclcpp std_srvs ariac_msgs)

# # add cpp executable here
# add_executable()
# ament_target_dependencies( rclcpp std_srvs std_msgs ariac_msgs)

# install targets
install(TARGETS
  #
  #
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
	#
  #
  #
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

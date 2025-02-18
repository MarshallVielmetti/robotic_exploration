cmake_minimum_required(VERSION 3.8)
project(sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)


find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# SETUP THE UNICYCLE MODEL
add_executable(unicycle_model_node src/UnicycleModelNode.cpp)
ament_target_dependencies(unicycle_model_node rclcpp geometry_msgs Eigen3)
target_include_directories(unicycle_model_node 
  PUBLIC 
  ${PROJECT_SOURCE_DIR}/include
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
endif()

install(TARGETS
  unicycle_model_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION share/${PROJECT_NAME}/install
)

# INSTALL VARIOUS SIMULATION FILES

install(DIRECTORY
	launch/
	DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
	urdf/
	DESTINATION share/${PROJECT_NAME}/urdf
)

install(DIRECTORY
	sdf/
	DESTINATION share/${PROJECT_NAME}/sdf
)

install(DIRECTORY
  models/
  DESTINATION share/${PROJECT_NAME}/models
)

install(DIRECTORY
  worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY
  rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

ament_package()

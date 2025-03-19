cmake_minimum_required(VERSION 3.8)
project(exploration_sim_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE Debug)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(TBB REQUIRED)

# custom packages
find_package(exploration_sim_msgs REQUIRED)

# set(Torch_DIR "/home/marshallvielmetti/Documents/torch/libtorch/share/cmake/Torch")
# find_package(Torch REQUIRED)
# include_directories(${TORCH_INCLUDE_DIRS})
# target_link_libraries(${PROJECT_NAME} ${TORCH_LIBRARIES})

add_executable(path_planner src/PathPlannerNode.cpp src/path_planner/AStarPathPlanner.cpp)
ament_target_dependencies(path_planner rclcpp nav_msgs geometry_msgs Eigen3)
target_include_directories(path_planner PRIVATE ${PROJECT_SOURCE_DIR}/include)

add_executable(pose_listener src/PoseTransformListenerNode.cpp)
ament_target_dependencies(pose_listener rclcpp geometry_msgs tf2 tf2_ros Eigen3)
target_include_directories(pose_listener PRIVATE ${PROJECT_SOURCE_DIR}/include)


add_executable(coverage_path_planner src/CoveragePathPlannerNode.cpp src/ConnectedComponentsLabeling.cpp)
ament_target_dependencies(coverage_path_planner rclcpp nav_msgs geometry_msgs Eigen3 exploration_sim_msgs)
target_link_libraries(coverage_path_planner TBB::tbb)
target_include_directories(coverage_path_planner PRIVATE ${PROJECT_SOURCE_DIR}/include)

# add_executable(trajectory_tracker src/TrajectoryTrackerNode.cpp src/trajectory_tracker/MPPITrajectoryTracker.cpp)
# ament_target_dependencies(trajectory_tracker rclcpp nav_msgs geometry_msgs Eigen3 Torch)
# target_include_directories(trajectory_tracker PRIVATE ${PROJECT_SOURCE_DIR}/include)


## INSTALL TARGETS ##
install(TARGETS 
  path_planner
  pose_listener
  coverage_path_planner
  DESTINATION lib/${PROJECT_NAME}
)


## INSTALL INCLUDE ## 
install(DIRECTORY include/
  DESTINATION share/${PROJECT_NAME}/include)


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

ament_package()

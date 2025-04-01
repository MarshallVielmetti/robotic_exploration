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
find_package(tf2_geometry_msgs REQUIRED)
find_package(TBB REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(ompl REQUIRED)

# Add these lines after finding OMPL
if(OMPL_FOUND)
  message(STATUS "Found OMPL: ${OMPL_INCLUDE_DIRS}")
  include_directories(${OMPL_INCLUDE_DIRS})
endif()

# custom packages
find_package(exploration_sim_msgs REQUIRED)

# set(Torch_DIR "/home/marshallvielmetti/Documents/torch/libtorch/share/cmake/Torch")
# find_package(Torch REQUIRED)
# include_directories(${TORCH_INCLUDE_DIRS})
# target_link_libraries(${PROJECT_NAME} ${TORCH_LIBRARIES})

# add_executable(path_planner src/PathPlannerNode.cpp src/path_planner/AStarPathPlanner.cpp)
# ament_target_dependencies(path_planner rclcpp nav_msgs geometry_msgs Eigen3)
# target_include_directories(path_planner PRIVATE ${PROJECT_SOURCE_DIR}/include)

# Compile Shared Utility Library
add_library(exploration_sim_planner_util SHARED
  src/util/SimulatedAnnealingSolver.cpp
  src/util/BSplineUtil.cpp
  src/util/EsdfUtil.cpp
  src/util/MpcController.cpp
  src/util/DubinsPath.cpp
)
ament_target_dependencies(exploration_sim_planner_util std_msgs geometry_msgs nav_msgs exploration_sim_msgs Eigen3 ompl)
target_link_libraries(exploration_sim_planner_util TBB::tbb ${OSQP_LIBRARIES} OsqpEigen::OsqpEigen ${OMPL_LIBRARIES})
target_include_directories(exploration_sim_planner_util PRIVATE ${PROJECT_SOURCE_DIR}/include OsqpEigen::OsqpEigen ${OMPL_INCLUDE_DIRS})

# Compile Pose Transform Listener
add_executable(pose_listener src/nodes/PoseTransformListenerNode.cpp)
ament_target_dependencies(pose_listener rclcpp geometry_msgs tf2 tf2_ros Eigen3)
target_include_directories(pose_listener PRIVATE ${PROJECT_SOURCE_DIR}/include)


# Compile Coverage Path Planner
add_library(coverage_path_planner_lib SHARED
  src/coverage_planner/ConnectedComponentsLabeling.cpp
)
ament_target_dependencies(coverage_path_planner_lib nav_msgs geometry_msgs Eigen3 exploration_sim_msgs)
target_include_directories(coverage_path_planner_lib PRIVATE ${PROJECT_SOURCE_DIR}/include)

add_executable(coverage_path_planner src/nodes/CoveragePathPlannerNode.cpp) 
ament_target_dependencies(coverage_path_planner rclcpp nav_msgs geometry_msgs Eigen3 exploration_sim_msgs)
target_link_libraries(coverage_path_planner TBB::tbb coverage_path_planner_lib exploration_sim_planner_util)
target_include_directories(coverage_path_planner PRIVATE ${PROJECT_SOURCE_DIR}/include)

# Compile short term planner
add_library(short_term_planner_lib SHARED
  src/short_term_planner/ShortTermPathPlanner.cpp
)
ament_target_dependencies(short_term_planner_lib nav_msgs geometry_msgs Eigen3 exploration_sim_msgs)
target_include_directories(short_term_planner_lib PRIVATE ${PROJECT_SOURCE_DIR}/include)

add_executable(short_term_planner src/nodes/ShortTermPlannerNode.cpp)
ament_target_dependencies(short_term_planner rclcpp nav_msgs geometry_msgs Eigen3 exploration_sim_msgs)
target_link_libraries(short_term_planner short_term_planner_lib exploration_sim_planner_util)
target_include_directories(short_term_planner PRIVATE ${PROJECT_SOURCE_DIR}/include)

# Compile trajectory tracker
add_library(trajectory_tracker_lib SHARED
  src/trajectory_tracker/PurePursuitTracker.cpp
  src/trajectory_tracker/MPCTrajectoryTracker.cpp
)
ament_target_dependencies(trajectory_tracker_lib nav_msgs geometry_msgs Eigen3)
target_include_directories(trajectory_tracker_lib PRIVATE ${PROJECT_SOURCE_DIR}/include)

add_executable(trajectory_tracker src/nodes/TrajectoryTrackerNode.cpp)
ament_target_dependencies(trajectory_tracker rclcpp std_msgs nav_msgs geometry_msgs Eigen3 tf2 tf2_ros tf2_geometry_msgs)
target_link_libraries(trajectory_tracker trajectory_tracker_lib exploration_sim_planner_util)
target_include_directories(trajectory_tracker PRIVATE ${PROJECT_SOURCE_DIR}/include)

# Compile Differential Drive Controller
add_executable(diff_drive_controller src/nodes/DiffDriveControllerNode.cpp)
ament_target_dependencies(diff_drive_controller rclcpp nav_msgs geometry_msgs std_msgs Eigen3)
target_include_directories(diff_drive_controller PRIVATE ${PROJECT_SOURCE_DIR}/include)



## INSTALL LIBRARIES ##
install(TARGETS 
  exploration_sim_planner_util
  coverage_path_planner_lib
  short_term_planner_lib
  trajectory_tracker_lib
  DESTINATION lib/${PROJECT_NAME}
)

## INSTALL TARGETS ##
install(TARGETS 
  pose_listener
  coverage_path_planner
  short_term_planner
  trajectory_tracker
  diff_drive_controller
  DESTINATION lib/${PROJECT_NAME}
)


## INSTALL INCLUDE ## 
install(DIRECTORY include/
  DESTINATION share/${PROJECT_NAME}/include)


if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test test/dubins_test.cpp)
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_test
    Eigen3
  )
  target_link_libraries(${PROJECT_NAME}_test exploration_sim_planner_util)

  add_executable(ompl_dubins_test test/ompl_dubins_test.cpp)
  ament_target_dependencies(ompl_dubins_test ompl)
  target_include_directories(ompl_dubins_test PRIVATE ${OMPL_INCLUDE_DIRS})
  target_link_libraries(ompl_dubins_test ${OMPL_LIBRARIES})
  ament_add_test(ompl_dubins_test
    COMMAND $<TARGET_FILE:ompl_dubins_test>
  )

  
  # MPC test
  add_executable(mpc_test test/mpc_test.cpp)
  target_link_libraries(mpc_test exploration_sim_planner_util)
  ament_target_dependencies(mpc_test Eigen3)
  target_include_directories(mpc_test PRIVATE ${PROJECT_SOURCE_DIR}/include)
  ament_add_test(mpc_test COMMAND $<TARGET_FILE:mpc_test>)
  

  # ament_lint_auto_find_test_dependencies()
endif()

ament_package()

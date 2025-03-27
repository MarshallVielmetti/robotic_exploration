cmake_minimum_required(VERSION 3.8)
project(exploration_sim_rviz)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(exploration_sim_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
  include/exploration_sim_rviz/frontier_display.hpp
  include/exploration_sim_rviz/connectivity_graph_display.hpp
)

add_library(frontier_display src/frontier_display.cpp src/connectivity_graph_display.cpp ${MOC_FILES})
target_include_directories(frontier_display PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(frontier_display
  pluginlib
  rviz_common
  exploration_sim_msgs
)

# add_library(connectivity_graph_display src/connectivity_graph_display.cpp ${MOC_FILES})
# target_include_directories(connectivity_graph_display PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

# ament_target_dependencies(connectivity_graph_display
#   pluginlib
#   rviz_common
#   exploration_sim_msgs
# )

install(TARGETS frontier_display 
  EXPORT export_exploration_sim_rviz
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
install(FILES rviz_common_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

ament_export_include_directories(include)
ament_export_targets(export_exploration_sim_rviz)
pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)

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

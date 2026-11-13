cmake_minimum_required(VERSION 3.0.2)
project(odom_to_mavros)

add_compile_options(-std=c++11)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geometry_msgs
  mavros_msgs
  nav_msgs
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS diagnostic_msgs geometry_msgs mavros_msgs nav_msgs roscpp std_msgs
)

add_executable(odom_to_mavros_node src/odom_to_mavros.cpp)
target_include_directories(odom_to_mavros_node PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(odom_to_mavros_node
  ${catkin_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
)
if(catkin_EXPORTED_TARGETS)
  add_dependencies(odom_to_mavros_node ${catkin_EXPORTED_TARGETS})
endif()

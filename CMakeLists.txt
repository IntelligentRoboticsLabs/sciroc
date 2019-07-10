cmake_minimum_required(VERSION 2.8.3)
project(sciroc)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  bica_planning
  bica
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sciroc
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# Actions
add_executable(rp_check_table_status src/actions/RP_check_table_status.cpp)
add_dependencies(rp_check_table_status ${catkin_EXPORTED_TARGETS})
target_link_libraries(rp_check_table_status ${catkin_LIBRARIES} )

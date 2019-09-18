cmake_minimum_required(VERSION 2.8.3)
project(sciroc)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  bica_planning
  bica_graph
  bica
  dialogflow_ros_msgs
  roslint
  darknet_ros_3d
  darknet_ros_3d_msgs
  gb_datahub
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP yaml-cpp REQUIRED)
if(YAMLCPP_VERSION VERSION_GREATER "0.5.0")
    add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
endif()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sciroc
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIRS}
)

# Utils
add_executable(df_simulator_restaurant_node src/utils/df_simulator_restaurant_node.cpp)
add_dependencies(df_simulator_restaurant_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(df_simulator_restaurant_node ${catkin_LIBRARIES})

# Actions Restaurant
add_executable(rp_check_table_status src/actions/RP_check_table_status.cpp)
add_dependencies(rp_check_table_status ${catkin_EXPORTED_TARGETS})
target_link_libraries(rp_check_table_status ${catkin_LIBRARIES} ${YAMLCPP_LIBRARIES})

add_executable(rp_get_order src/actions/RP_get_order.cpp)
add_dependencies(rp_get_order ${catkin_EXPORTED_TARGETS})
target_link_libraries(rp_get_order ${catkin_LIBRARIES} ${YAMLCPP_LIBRARIES})

add_executable(rp_set_order src/actions/RP_set_order.cpp)
add_dependencies(rp_set_order ${catkin_EXPORTED_TARGETS})
target_link_libraries(rp_set_order ${catkin_LIBRARIES} ${YAMLCPP_LIBRARIES})

add_executable(rp_check_order src/actions/RP_check_order.cpp)
add_dependencies(rp_check_order ${catkin_EXPORTED_TARGETS})
target_link_libraries(rp_check_order ${catkin_LIBRARIES} ${YAMLCPP_LIBRARIES})

add_executable(rp_fix_order src/actions/RP_fix_order.cpp)
add_dependencies(rp_fix_order ${catkin_EXPORTED_TARGETS})
target_link_libraries(rp_fix_order ${catkin_LIBRARIES} ${YAMLCPP_LIBRARIES})

add_executable(rp_deliver_order src/actions/RP_deliver_order.cpp)
add_dependencies(rp_deliver_order ${catkin_EXPORTED_TARGETS})
target_link_libraries(rp_deliver_order ${catkin_LIBRARIES} ${YAMLCPP_LIBRARIES})

#add_executable(rp_gretting_new_customer src/actions/RP_gretting_new_customer.cpp)
#add_dependencies(rp_gretting_new_customer ${catkin_EXPORTED_TARGETS})
#target_link_libraries(rp_gretting_new_customer ${catkin_LIBRARIES})

add_executable(rp_check_waiting_person src/actions/RP_check_waiting_person.cpp)
add_dependencies(rp_check_waiting_person ${catkin_EXPORTED_TARGETS})
target_link_libraries(rp_check_waiting_person ${catkin_LIBRARIES})

add_executable(rp_attend_person src/actions/RP_attend_person.cpp)
add_dependencies(rp_attend_person ${catkin_EXPORTED_TARGETS})
target_link_libraries(rp_attend_person ${catkin_LIBRARIES})


# Actions Elevator

add_executable(rp_move_to_floor src/actions/RP_move_to_floor.cpp)
add_dependencies(rp_move_to_floor ${catkin_EXPORTED_TARGETS})
target_link_libraries(rp_move_to_floor ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}
  src/sciroc/Utils.cpp
)

# HFSM

add_executable(restaurant_executor_node
  src/restaurant/restaurant_executor_node.cpp
  src/restaurant/restaurant_executor.cpp
  src/restaurant/restaurant_hfsm.cpp
)
add_dependencies(restaurant_executor_node
  ${restaurant_executor_node_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(restaurant_executor_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

add_executable(elevator_executor_node
  src/elevator/elevator_executor_node.cpp
  src/elevator/elevator_executor.cpp
  src/elevator/elevator_hfsm.cpp
)
add_dependencies(elevator_executor_node
  ${elevator_executor_node_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(elevator_executor_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)



# Tests Restaurant

add_executable(restaurant_test_check_table src/restaurant/tests/restaurant_test_check_table.cpp)
add_dependencies(restaurant_test_check_table ${catkin_EXPORTED_TARGETS})
target_link_libraries(restaurant_test_check_table ${catkin_LIBRARIES} ${YAMLCPP_LIBRARIES})

add_executable(restaurant_test_get_order src/restaurant/tests/restaurant_test_get_order.cpp)
add_dependencies(restaurant_test_get_order ${catkin_EXPORTED_TARGETS})
target_link_libraries(restaurant_test_get_order ${catkin_LIBRARIES} ${YAMLCPP_LIBRARIES})

add_executable(restaurant_test_set_order src/restaurant/tests/restaurant_test_set_order.cpp)
add_dependencies(restaurant_test_set_order ${catkin_EXPORTED_TARGETS})
target_link_libraries(restaurant_test_set_order ${catkin_LIBRARIES} ${YAMLCPP_LIBRARIES})

add_executable(restaurant_test_check_order src/restaurant/tests/restaurant_test_check_order.cpp)
add_dependencies(restaurant_test_check_order ${catkin_EXPORTED_TARGETS})
target_link_libraries(restaurant_test_check_order ${catkin_LIBRARIES} ${YAMLCPP_LIBRARIES})

add_executable(restaurant_test_waiting_zone src/restaurant/tests/restaurant_test_waiting_zone.cpp)
add_dependencies(restaurant_test_waiting_zone ${catkin_EXPORTED_TARGETS})
target_link_libraries(restaurant_test_waiting_zone ${catkin_LIBRARIES} ${YAMLCPP_LIBRARIES})

add_executable(restaurant_test_attend_person src/restaurant/tests/restaurant_test_attend_person.cpp)
add_dependencies(restaurant_test_attend_person ${catkin_EXPORTED_TARGETS})
target_link_libraries(restaurant_test_attend_person ${catkin_LIBRARIES} ${YAMLCPP_LIBRARIES})

# Tests Elevator

add_executable(elevator_test_social_move src/elevator/tests/elevator_test_social_move.cpp)
add_dependencies(elevator_test_social_move ${catkin_EXPORTED_TARGETS})
target_link_libraries(elevator_test_social_move ${catkin_LIBRARIES} ${YAMLCPP_LIBRARIES} ${PROJECT_NAME})

add_executable(elevator_test_move_to_floor src/elevator/tests/elevator_test_move_to_floor.cpp)
add_dependencies(elevator_test_move_to_floor ${catkin_EXPORTED_TARGETS})
target_link_libraries(elevator_test_move_to_floor ${catkin_LIBRARIES} ${YAMLCPP_LIBRARIES} ${PROJECT_NAME})

add_executable(elevator_test_proxemic_move src/elevator/tests/elevator_test_proxemic_move.cpp)
add_dependencies(elevator_test_proxemic_move ${catkin_EXPORTED_TARGETS})
target_link_libraries(elevator_test_proxemic_move ${catkin_LIBRARIES} ${YAMLCPP_LIBRARIES} ${PROJECT_NAME})

set(ROSLINT_CPP_OPTS "--filter=-legal/copyright")

roslint_cpp(
  src/utils/df_simulator_restaurant_node.cpp
  src/actions/RP_check_table_status.cpp
  src/actions/RP_get_order.cpp src/actions/RP_get_order.h
  src/actions/RP_set_order.cpp src/actions/RP_set_order.h
  src/actions/RP_check_order.cpp src/actions/RP_check_order.h
  src/actions/RP_fix_order.cpp src/actions/RP_fix_order.h
  src/actions/RP_deliver_order.cpp src/actions/RP_deliver_order.h
  src/actions/RP_check_waiting_person.cpp src/actions/RP_check_waiting_person.h
  src/actions/RP_attend_person.cpp src/actions/RP_attend_person.h
  src/restaurant/restaurant_executor.cpp src/restaurant/restaurant_executor.h
  src/elevator/elevator_executor.cpp src/elevator/elevator_executor.h

)

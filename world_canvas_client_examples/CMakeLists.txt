cmake_minimum_required(VERSION 2.8.3)
project(world_canvas_client_examples)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  unique_id
  uuid_msgs
  nav_msgs
  yocs_msgs
  world_canvas_msgs
  world_canvas_client_cpp
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(test_world_canvas_client src/test_annotation_collection.cpp)
target_link_libraries(test_world_canvas_client ${catkin_LIBRARIES})

install(TARGETS test_world_canvas_client DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

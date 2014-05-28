cmake_minimum_required(VERSION 2.8.3)
project(world_canvas_client_cpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  world_canvas_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES world_canvas_client
  CATKIN_DEPENDS roscpp world_canvas_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(world_canvas_client src/world_canvas_client.cpp)
add_dependencies(world_canvas_client world_canvas_msgs_gencpp)
target_link_libraries(world_canvas_client uuid ${catkin_LIBRARIES})

install(TARGETS world_canvas_client LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION include FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")

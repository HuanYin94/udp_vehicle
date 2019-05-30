cmake_minimum_required(VERSION 2.8.3)
project(udp_vehicle)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  tf
  tf_conversions
)

add_definitions(-std=c++11 -O2 -g -DNDEBUG)

catkin_package (
#	CATKIN_DEPENDS roscpp
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}

)

add_executable(server src/server.cpp)
target_link_libraries(server
${catkin_LIBRARIES}
)

add_executable(sender src/sender.cpp)
target_link_libraries(sender
${catkin_LIBRARIES}
)







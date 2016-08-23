cmake_minimum_required(VERSION 2.8.3)
project(planification)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES planification
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(planification-node src/planification.cpp)
add_dependencies(planification-node planification_generate_messages_cpp)

TARGET_LINK_LIBRARIES(planification-node -L/opt/ros/hydro/lib opencv_core opencv_highgui opencv_imgproc opencv_video ${catkin_LIBRARIES})


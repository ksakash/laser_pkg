cmake_minimum_required (VERSION 3.0.2)
project (laser_pkg)

# add_compile_options(-std=c++11)

find_package (catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    geometry_msgs
    std_msgs
    message_generation
)

find_package (gazebo REQUIRED)

add_message_files (
  FILES
  Laser3d.msg
  PointArray.msg
  ImagePose.msg
)

catkin_python_setup()

generate_messages (
  DEPENDENCIES
  std_msgs geometry_msgs sensor_msgs
)

catkin_package (
    CATKIN_DEPENDS roscpp sensor_msgs std_msgs geometry_msgs message_generation
)

list (APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

include_directories (
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories (${GAZEBO_LIBRARY_DIRS})

add_executable (node src/node.cpp)
target_link_libraries (node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} pthread)

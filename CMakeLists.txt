cmake_minimum_required(VERSION 2.6.3)

project(lcsr_tf_tools)

find_package(catkin REQUIRED tf geometry_msgs interactive_markers visualization_msgs message_generation)

add_message_files(
  FILES StaticTransform.msg)

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(interactive_transform_publisher src/interactive_transform_publisher.cpp)
target_link_libraries(interactive_transform_publisher ${catkin_LIBRARIES})

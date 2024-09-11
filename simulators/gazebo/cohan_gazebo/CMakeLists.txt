cmake_minimum_required(VERSION 2.8.3)
project(cohan_gazebo)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  nav_msgs
)

add_message_files( DIRECTORY msg
  FILES
    setGoal.msg
    abortGoal.msg
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    nav_msgs
    std_msgs
)


catkin_package()

include_directories()



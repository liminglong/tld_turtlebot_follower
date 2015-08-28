cmake_minimum_required(VERSION 2.8.3)
project(tld_turtlebot_follower)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tld_msgs
  kobuki_msgs
)


# add_message_files(
 #  FILES
  # tld_tracked_object.msg   
# )
generate_messages(DEPENDENCIES std_msgs geometry_msgs kobuki_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tld_turtlebot_follower
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs tld_msgs kobuki_msgs

)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(tld_turtlebot_follower src/tld_turtlebot_follower.cpp)
target_link_libraries(tld_turtlebot_follower ${catkin_LIBRARIES})
add_dependencies(tld_turtlebot_follower tld_turtlebot_follower_generate_messages_cpp)



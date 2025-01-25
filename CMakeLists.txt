cmake_minimum_required(VERSION 2.8.3)
project(yangda_camera_control)




find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  sensor_msgs
  std_msgs
  message_generation
)
add_message_files(
  FILES
  gimbal_feedback.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    serial
    sensor_msgs
    std_msgs
    message_runtime
)



include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(yangda_camera_control src/yangda_camera_control.cpp)



target_link_libraries(yangda_camera_control
  ${catkin_LIBRARIES}
)


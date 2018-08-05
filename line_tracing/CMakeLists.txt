cmake_minimum_required(VERSION 2.8.3)
project(line_tracing)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)
find_package(OpenCV REQUIRED)


################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(
  FILES
  hsv_thresh.msg
  theta_stamped.msg
)

generate_messages(DEPENDENCIES std_msgs)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################
catkin_package(
##  INCLUDE_DIRS include
##  LIBRARIES cv_bridge_cpp
##  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
##  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(line_tracing_node src/line_tracing_node.cpp)
target_link_libraries(line_tracing_node ${catkin_LIBRARIES} ${OpenCV_LIBS})


#############
## Install ##
#############


#############
## Testing ##
#############

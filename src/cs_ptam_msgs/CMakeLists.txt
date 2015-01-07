cmake_minimum_required(VERSION 2.4.6)
project(cs_ptam_msgs)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS message_generation sensor_msgs)

# Generate Messages & Services
add_message_files(FILES
    Corner.msg DepthImage.msg Edge.msg Edges.msg Keyframe.msg Level.msg Mappoint.msg RGBImage.msg SharedImageSet.msg)

add_service_files(FILES
  InitStep.srv
) 

generate_messages(DEPENDENCIES sensor_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)

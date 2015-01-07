# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "cs_ptam_msgs: 9 messages, 1 services")

set(MSG_I_FLAGS "-Ics_ptam_msgs:/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg;-Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(cs_ptam_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/srv/InitStep.srv" NAME_WE)
add_custom_target(_cs_ptam_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cs_ptam_msgs" "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/srv/InitStep.srv" ""
)

get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Edges.msg" NAME_WE)
add_custom_target(_cs_ptam_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cs_ptam_msgs" "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Edges.msg" "geometry_msgs/Point:geometry_msgs/Pose:geometry_msgs/Quaternion:std_msgs/Header:cs_ptam_msgs/Edge"
)

get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Corner.msg" NAME_WE)
add_custom_target(_cs_ptam_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cs_ptam_msgs" "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Corner.msg" ""
)

get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Level.msg" NAME_WE)
add_custom_target(_cs_ptam_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cs_ptam_msgs" "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Level.msg" "cs_ptam_msgs/Corner"
)

get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Edge.msg" NAME_WE)
add_custom_target(_cs_ptam_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cs_ptam_msgs" "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Edge.msg" "geometry_msgs/Quaternion:geometry_msgs/Point:geometry_msgs/Pose"
)

get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Keyframe.msg" NAME_WE)
add_custom_target(_cs_ptam_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cs_ptam_msgs" "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Keyframe.msg" "geometry_msgs/Point:cs_ptam_msgs/RGBImage:cs_ptam_msgs/Level:cs_ptam_msgs/DepthImage:geometry_msgs/Quaternion:cs_ptam_msgs/Corner:cs_ptam_msgs/Mappoint:geometry_msgs/Pose:std_msgs/Header"
)

get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/DepthImage.msg" NAME_WE)
add_custom_target(_cs_ptam_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cs_ptam_msgs" "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/DepthImage.msg" ""
)

get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/RGBImage.msg" NAME_WE)
add_custom_target(_cs_ptam_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cs_ptam_msgs" "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/RGBImage.msg" ""
)

get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Mappoint.msg" NAME_WE)
add_custom_target(_cs_ptam_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cs_ptam_msgs" "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Mappoint.msg" ""
)

get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/SharedImageSet.msg" NAME_WE)
add_custom_target(_cs_ptam_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cs_ptam_msgs" "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/SharedImageSet.msg" "std_msgs/Header"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(cs_ptam_msgs
  "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Edges.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Edge.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cs_ptam_msgs
)
_generate_msg_cpp(cs_ptam_msgs
  "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Corner.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cs_ptam_msgs
)
_generate_msg_cpp(cs_ptam_msgs
  "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Level.msg"
  "${MSG_I_FLAGS}"
  "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Corner.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cs_ptam_msgs
)
_generate_msg_cpp(cs_ptam_msgs
  "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Edge.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cs_ptam_msgs
)
_generate_msg_cpp(cs_ptam_msgs
  "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Keyframe.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/RGBImage.msg;/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Level.msg;/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/DepthImage.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Corner.msg;/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Mappoint.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cs_ptam_msgs
)
_generate_msg_cpp(cs_ptam_msgs
  "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/DepthImage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cs_ptam_msgs
)
_generate_msg_cpp(cs_ptam_msgs
  "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/RGBImage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cs_ptam_msgs
)
_generate_msg_cpp(cs_ptam_msgs
  "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Mappoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cs_ptam_msgs
)
_generate_msg_cpp(cs_ptam_msgs
  "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/SharedImageSet.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cs_ptam_msgs
)

### Generating Services
_generate_srv_cpp(cs_ptam_msgs
  "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/srv/InitStep.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cs_ptam_msgs
)

### Generating Module File
_generate_module_cpp(cs_ptam_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cs_ptam_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(cs_ptam_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(cs_ptam_msgs_generate_messages cs_ptam_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/srv/InitStep.srv" NAME_WE)
add_dependencies(cs_ptam_msgs_generate_messages_cpp _cs_ptam_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Edges.msg" NAME_WE)
add_dependencies(cs_ptam_msgs_generate_messages_cpp _cs_ptam_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Corner.msg" NAME_WE)
add_dependencies(cs_ptam_msgs_generate_messages_cpp _cs_ptam_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Level.msg" NAME_WE)
add_dependencies(cs_ptam_msgs_generate_messages_cpp _cs_ptam_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Edge.msg" NAME_WE)
add_dependencies(cs_ptam_msgs_generate_messages_cpp _cs_ptam_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Keyframe.msg" NAME_WE)
add_dependencies(cs_ptam_msgs_generate_messages_cpp _cs_ptam_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/DepthImage.msg" NAME_WE)
add_dependencies(cs_ptam_msgs_generate_messages_cpp _cs_ptam_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/RGBImage.msg" NAME_WE)
add_dependencies(cs_ptam_msgs_generate_messages_cpp _cs_ptam_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Mappoint.msg" NAME_WE)
add_dependencies(cs_ptam_msgs_generate_messages_cpp _cs_ptam_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/SharedImageSet.msg" NAME_WE)
add_dependencies(cs_ptam_msgs_generate_messages_cpp _cs_ptam_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cs_ptam_msgs_gencpp)
add_dependencies(cs_ptam_msgs_gencpp cs_ptam_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cs_ptam_msgs_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(cs_ptam_msgs
  "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Edges.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Edge.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cs_ptam_msgs
)
_generate_msg_lisp(cs_ptam_msgs
  "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Corner.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cs_ptam_msgs
)
_generate_msg_lisp(cs_ptam_msgs
  "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Level.msg"
  "${MSG_I_FLAGS}"
  "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Corner.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cs_ptam_msgs
)
_generate_msg_lisp(cs_ptam_msgs
  "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Edge.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cs_ptam_msgs
)
_generate_msg_lisp(cs_ptam_msgs
  "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Keyframe.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/RGBImage.msg;/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Level.msg;/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/DepthImage.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Corner.msg;/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Mappoint.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cs_ptam_msgs
)
_generate_msg_lisp(cs_ptam_msgs
  "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/DepthImage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cs_ptam_msgs
)
_generate_msg_lisp(cs_ptam_msgs
  "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/RGBImage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cs_ptam_msgs
)
_generate_msg_lisp(cs_ptam_msgs
  "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Mappoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cs_ptam_msgs
)
_generate_msg_lisp(cs_ptam_msgs
  "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/SharedImageSet.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cs_ptam_msgs
)

### Generating Services
_generate_srv_lisp(cs_ptam_msgs
  "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/srv/InitStep.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cs_ptam_msgs
)

### Generating Module File
_generate_module_lisp(cs_ptam_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cs_ptam_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(cs_ptam_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(cs_ptam_msgs_generate_messages cs_ptam_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/srv/InitStep.srv" NAME_WE)
add_dependencies(cs_ptam_msgs_generate_messages_lisp _cs_ptam_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Edges.msg" NAME_WE)
add_dependencies(cs_ptam_msgs_generate_messages_lisp _cs_ptam_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Corner.msg" NAME_WE)
add_dependencies(cs_ptam_msgs_generate_messages_lisp _cs_ptam_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Level.msg" NAME_WE)
add_dependencies(cs_ptam_msgs_generate_messages_lisp _cs_ptam_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Edge.msg" NAME_WE)
add_dependencies(cs_ptam_msgs_generate_messages_lisp _cs_ptam_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Keyframe.msg" NAME_WE)
add_dependencies(cs_ptam_msgs_generate_messages_lisp _cs_ptam_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/DepthImage.msg" NAME_WE)
add_dependencies(cs_ptam_msgs_generate_messages_lisp _cs_ptam_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/RGBImage.msg" NAME_WE)
add_dependencies(cs_ptam_msgs_generate_messages_lisp _cs_ptam_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Mappoint.msg" NAME_WE)
add_dependencies(cs_ptam_msgs_generate_messages_lisp _cs_ptam_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/SharedImageSet.msg" NAME_WE)
add_dependencies(cs_ptam_msgs_generate_messages_lisp _cs_ptam_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cs_ptam_msgs_genlisp)
add_dependencies(cs_ptam_msgs_genlisp cs_ptam_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cs_ptam_msgs_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(cs_ptam_msgs
  "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Edges.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Edge.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cs_ptam_msgs
)
_generate_msg_py(cs_ptam_msgs
  "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Corner.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cs_ptam_msgs
)
_generate_msg_py(cs_ptam_msgs
  "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Level.msg"
  "${MSG_I_FLAGS}"
  "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Corner.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cs_ptam_msgs
)
_generate_msg_py(cs_ptam_msgs
  "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Edge.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cs_ptam_msgs
)
_generate_msg_py(cs_ptam_msgs
  "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Keyframe.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/RGBImage.msg;/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Level.msg;/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/DepthImage.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Corner.msg;/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Mappoint.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cs_ptam_msgs
)
_generate_msg_py(cs_ptam_msgs
  "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/DepthImage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cs_ptam_msgs
)
_generate_msg_py(cs_ptam_msgs
  "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/RGBImage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cs_ptam_msgs
)
_generate_msg_py(cs_ptam_msgs
  "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Mappoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cs_ptam_msgs
)
_generate_msg_py(cs_ptam_msgs
  "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/SharedImageSet.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cs_ptam_msgs
)

### Generating Services
_generate_srv_py(cs_ptam_msgs
  "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/srv/InitStep.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cs_ptam_msgs
)

### Generating Module File
_generate_module_py(cs_ptam_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cs_ptam_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(cs_ptam_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(cs_ptam_msgs_generate_messages cs_ptam_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/srv/InitStep.srv" NAME_WE)
add_dependencies(cs_ptam_msgs_generate_messages_py _cs_ptam_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Edges.msg" NAME_WE)
add_dependencies(cs_ptam_msgs_generate_messages_py _cs_ptam_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Corner.msg" NAME_WE)
add_dependencies(cs_ptam_msgs_generate_messages_py _cs_ptam_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Level.msg" NAME_WE)
add_dependencies(cs_ptam_msgs_generate_messages_py _cs_ptam_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Edge.msg" NAME_WE)
add_dependencies(cs_ptam_msgs_generate_messages_py _cs_ptam_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Keyframe.msg" NAME_WE)
add_dependencies(cs_ptam_msgs_generate_messages_py _cs_ptam_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/DepthImage.msg" NAME_WE)
add_dependencies(cs_ptam_msgs_generate_messages_py _cs_ptam_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/RGBImage.msg" NAME_WE)
add_dependencies(cs_ptam_msgs_generate_messages_py _cs_ptam_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/Mappoint.msg" NAME_WE)
add_dependencies(cs_ptam_msgs_generate_messages_py _cs_ptam_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/young/localhome/code/idSLAM/src/cs_ptam_msgs/msg/SharedImageSet.msg" NAME_WE)
add_dependencies(cs_ptam_msgs_generate_messages_py _cs_ptam_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cs_ptam_msgs_genpy)
add_dependencies(cs_ptam_msgs_genpy cs_ptam_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cs_ptam_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cs_ptam_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cs_ptam_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(cs_ptam_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cs_ptam_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cs_ptam_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(cs_ptam_msgs_generate_messages_lisp sensor_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cs_ptam_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cs_ptam_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cs_ptam_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(cs_ptam_msgs_generate_messages_py sensor_msgs_generate_messages_py)

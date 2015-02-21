# Put this line before the executable or library:
# initilaize boost directory search
rosbuild_add_boost_directories()

find_package(cmake_modules REQUIRED)
#find_package(bullet REQUIRED)

include_directories(   ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/src/backend/
    /usr/local/include/bullet)

rosbuild_add_library(nodebase BaseSLAMNode.cpp)
rosbuild_add_library(conversions conversions/conversions.cpp)
rosbuild_add_library(mapvis MapVisualization.cpp geometry/PointClouds.cpp)
rosbuild_add_library(ODT
    ODT/landing_object.cpp
#    ODT/myfeature_descriptor.cpp
#    ODT/mymatcher.cpp
#    ODT/conversions_img.cpp
    ODT/polygon.cpp
    ODT/cvdopencv_helper.cpp
    ODT/homographysolver.cpp
    ODT/padtracker.cpp
    ODT/MyOrb.cpp
)

set(COMMON_LIBRARIES nodebase ODT mapvis ptam backend conversions GVars3 cvd toontag)

add_subdirectory(backend)
add_subdirectory(ptam)
add_subdirectory(tools)
#add_subdirectory(cs_ptam_msgs)
add_subdirectory(geometry)
#rosbuild_add_executable(rgbd_ptam_node RGBDPTAMNode.cpp)
#target_link_libraries(rgbd_ptam_node ${COMMON_LIBRARIES})

#rosbuild_add_executable(stereo_ptam_node StereoPTAMNode.cpp)
#target_link_libraries(stereo_ptam_node ${COMMON_LIBRARIES})

#rosbuild_add_executable(mono_ptam_node MonoPTAMNode.cpp)
#target_link_libraries(mono_ptam_node ${COMMON_LIBRARIES})

#rosbuild_add_executable(mono_landing_node MonoLandingNode.cpp)
rosbuild_add_library(idslamnodelet idslamnodelet.cpp)
target_link_libraries(idslamnodelet ${COMMON_LIBRARIES})
rosbuild_link_boost(idslamnodelet signals)
#rosbuild_add_executable(dualmono_nodelet DualMonoSLAMnodelet.cpp)
#target_link_libraries(dualmono_nodelet ${COMMON_LIBRARIES})
# Put this line after the executable or library:
# add a given boost library to be linked with your target
#rosbuild_link_boost(dualmono_nodelet signals)

rosbuild_add_executable(cameras_calib Cameras_calib.cpp)
target_link_libraries(cameras_calib ${COMMON_LIBRARIES})

rosbuild_add_executable(kinect_calib kinect_calib.cpp)
target_link_libraries(kinect_calib ${COMMON_LIBRARIES})

#rosbuild_add_executable(rgbd_ptam_offline RGBDPTAMOffline.cpp)
#target_link_libraries(rgbd_ptam_offline ${COMMON_LIBRARIES})

# Testing application for camera model
#rosbuild_add_executable(camtest camtest.cpp)
#target_link_libraries(camtest ptam GVars3 cvd)

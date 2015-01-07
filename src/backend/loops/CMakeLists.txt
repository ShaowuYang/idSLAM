rosbuild_add_executable(creatvoc creatvoc.cpp)
target_link_libraries(creatvoc ${OpenCV_LIBS} ${CSLAM_COMMON_LIBRARIES})

add_library(cslam_loops LoopDetector.cpp)
target_link_libraries(cslam_loops ${OpenCV_LIBS})

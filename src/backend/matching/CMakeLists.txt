add_library(cslam_matching BruteForceMatcher.cpp)
target_link_libraries(cslam_matching ${OpenCV_LIBS} cslam_reg)

add_library(cslam_slam
  Keyframe.cpp
  PGOptimizer.cpp
  PlaneEdge.cpp
  SLAMSystem.cpp
)
target_link_libraries(cslam_slam ${OpenCV_LIBS})

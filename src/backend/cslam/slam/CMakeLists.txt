find_package(G2O REQUIRED)
set(G2O_LIBRARIES
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_SIM3}
)

add_library(cslam_slam
#  Keyframe.cpp
  PGOptimizer.cpp
  PlaneEdge.cpp
  SLAMSystem.cpp
)

target_link_libraries(cslam_slam
  ${G2O_LIBRARIES}
  ${SuiteSparse_LIBRARY}
  ${OpenCV_LIBS})

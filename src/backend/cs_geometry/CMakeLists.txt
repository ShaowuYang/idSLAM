
find_package(cmake_modules REQUIRED)
#find_package(catkin REQUIRED COMPONENTS cv_bridge roscpp tf pcl_ros)
find_package(Eigen REQUIRED)
find_package(OpenCV 2.4 REQUIRED)
find_package(PCL 1.6 REQUIRED COMPONENTS io)
find_package(Sophus REQUIRED)

include_directories(
    ${EIGEN_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)

set(EXTERNAL_LIBS
   ${OpenCV_LIBS}
   ${Boost_LIBRARIES}
   ${PCL_LIBRARIES}
   gsl gslcblas yaml-cpp
)

rosbuild_add_library(cs_geometry
  Camera.cpp
  Conversions.cpp
  Math.cpp
)
target_link_libraries(cs_geometry ${EXTERNAL_LIBS})

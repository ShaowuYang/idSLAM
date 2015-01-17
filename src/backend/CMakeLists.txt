set(CMAKE_BUILD_TYPE RelWithDebInfo)
add_definitions(-O3 -march=native -DCVD_HAVE_XMMINTRIN) # optimization
add_definitions(-fno-omit-frame-pointer) # required for profiling

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Boost 1.40 COMPONENTS filesystem system serialization thread REQUIRED)
find_package(DBoW2 REQUIRED)
find_package(Eigen REQUIRED)
find_package(G2O REQUIRED)
find_package(OpenCV 2.3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(SuiteSparse REQUIRED)
find_package(OpenMP REQUIRED)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

include_directories(
  #${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIR}
  ${DBoW2_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${G2O_INCLUDE_DIR}
  ${CSPARSE_INCLUDE_DIR}
)

set(G2O_LIBRARIES
  ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_SOLVER_CSPARSE} ${G2O_TYPES_SLAM3D} ${G2O_TYPES_SIM3}
)

set(EXTERNAL_LIBRARIES
  ${OpenCV_LIBS}
  ${DBoW2_LIBRARIES}
  ${Boost_LIBRARIES}
  #${CSPARSE_LIBRARY}
  ${G2O_LIBRARIES}
  ${SuiteSparse_LIBRARY}
  yaml-cpp cholmod #${catkin_LIBRARIES}
)

set(BACKEND_COMMON_LIBRARIES
  backend_slam
  backend_loops
  backend_reg
  backend_tools
  cs_geometry
  g2o_types
  ${EXTERNAL_LIBRARIES}
)

add_subdirectory(cs_geometry)
add_subdirectory(tools)
add_subdirectory(registration)
add_subdirectory(loops)
add_subdirectory(slam)
add_subdirectory(g2o_types)

add_library(backend
    backend.cpp
)

target_link_libraries(backend
    ${BACKEND_COMMON_LIBRARIES}
)

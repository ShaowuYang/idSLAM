rosbuild_add_executable(creatvoc creatvoc.cpp)
target_link_libraries(creatvoc ${OpenCV_LIBS} ${BACKEND_COMMON_LIBRARIES})

add_library(backend_loops LoopDetector.cpp)
target_link_libraries(backend_loops ${OpenCV_LIBS})

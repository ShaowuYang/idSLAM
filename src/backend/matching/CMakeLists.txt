add_library(backend_matching BruteForceMatcher.cpp)
target_link_libraries(backend_matching ${OpenCV_LIBS} backend_reg)

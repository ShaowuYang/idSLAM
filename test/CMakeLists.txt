include_directories(../src)

rosbuild_add_gtest(CameraTest CameraTest.cpp)

target_link_libraries(CameraTest
    ptam)

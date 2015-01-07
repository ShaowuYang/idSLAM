add_executable(cslam_node CSLAMNode.cpp cslam_node.cpp)
target_link_libraries(cslam_node ${COMMON_LIBRARIES})

add_executable(cslam_offline CSLAMNode.cpp cslam_offline.cpp)
target_link_libraries(cslam_offline ${COMMON_LIBRARIES})

add_executable(cslam_voc_gen voc_gen.cpp)
target_link_libraries(cslam_voc_gen ${COMMON_LIBRARIES})

add_executable(cslam_ba_alone ba_alone.cpp)
target_link_libraries(cslam_ba_alone ${COMMON_LIBRARIES})

add_executable(cslam_kf2pcd kf2pcd.cpp)
target_link_libraries(cslam_kf2pcd ${COMMON_LIBRARIES})

add_executable(cslam_eval_reg evaluate_registration.cpp)
target_link_libraries(cslam_eval_reg ${COMMON_LIBRARIES})

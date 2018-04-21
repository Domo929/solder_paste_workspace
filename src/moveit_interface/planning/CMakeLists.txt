add_executable(planning_scene src/planning_scene.cpp)
target_link_libraries(planning_scene ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS planning_scene DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(planning_scene_ros_api src/planning_scene_ros_api.cpp)
target_link_libraries(planning_scene_ros_api ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS planning_scene_ros_api DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(motion_planning_api src/motion_planning_api.cpp)
target_link_libraries(motion_planning_api ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS motion_planning_api DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(planning_pipeline src/planning_pipeline.cpp)
target_link_libraries(planning_pipeline ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS planning_pipeline DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(move_group_interface src/move_group_interface.cpp)
target_link_libraries(move_group_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_interface DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


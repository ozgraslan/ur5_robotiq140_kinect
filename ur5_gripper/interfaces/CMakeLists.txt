add_executable(ur5_cpp_test src/ur5_gripper_interface.cpp)
target_link_libraries(ur5_cpp_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ur5_cpp_test DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
cmake_minimum_required(VERSION 2.8.3)

project(epson_spelp_driver)

find_package(catkin REQUIRED)

catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(tests/roslaunch_test.xml
    DEPENDENCIES
      # not nice, but industrial_robot_client doesn't prefix its targets
      joint_trajectory_action
      motion_streaming_interface
      robot_state
  )
endif()

foreach(dir spel+ launch)
  install(
    DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()

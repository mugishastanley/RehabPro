# CMake generated Testfile for 
# Source directory: /home/adminuser/ws_moveit3/src/industrial_core/industrial_robot_client
# Build directory: /home/adminuser/ws_moveit3/build/industrial_robot_client
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_industrial_robot_client_gtest_utest_robot_client "/home/adminuser/ws_moveit3/build/industrial_robot_client/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/adminuser/ws_moveit3/build/industrial_robot_client/test_results/industrial_robot_client/gtest-utest_robot_client.xml" "--return-code" "/home/adminuser/ws_moveit3/devel/lib/industrial_robot_client/utest_robot_client --gtest_output=xml:/home/adminuser/ws_moveit3/build/industrial_robot_client/test_results/industrial_robot_client/gtest-utest_robot_client.xml")
add_test(_ctest_industrial_robot_client_roslaunch-check_test_roslaunch_test.xml "/home/adminuser/ws_moveit3/build/industrial_robot_client/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/adminuser/ws_moveit3/build/industrial_robot_client/test_results/industrial_robot_client/roslaunch-check_test_roslaunch_test.xml.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/adminuser/ws_moveit3/build/industrial_robot_client/test_results/industrial_robot_client" "/opt/ros/melodic/share/roslaunch/cmake/../scripts/roslaunch-check -o \"/home/adminuser/ws_moveit3/build/industrial_robot_client/test_results/industrial_robot_client/roslaunch-check_test_roslaunch_test.xml.xml\" \"/home/adminuser/ws_moveit3/src/industrial_core/industrial_robot_client/test/roslaunch_test.xml\" ")
subdirs("gtest")

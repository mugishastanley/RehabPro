cmake_minimum_required(VERSION 3.0.2)
project(motion_planning)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometric_shapes
  geometry_msgs
  interactive_markers
  moveit_core
  moveit_ros_perception
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  pcl_conversions
  pcl_ros
  pluginlib
  rosbag
  roscpp
  rospy
  std_msgs
  tf
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   std_msgs#   tf2_geometry_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES motion_planning
#  CATKIN_DEPENDS geometric_shapes geometry_msgs interactive_markers moveit_core moveit_ros_perception moveit_ros_planning moveit_ros_planning_interface pcl_conversions pcl_ros pluginlib rosbag roscpp rospy std_msgs tf tf2 tf2_eigen tf2_geometry_msgs tf2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/motion_planning.cpp
# )

#add_library(universal_robot
#  src/universal_robot/ur_kinematics/include/ur_kinematics/ur_kin.h
#  src/universal_robot/ur_kinematics/include/ur_kinematics/ikfast.h
#)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/motion_planning_node.cpp)

add_executable(move_jSpace src/move_jSpace.cpp)
add_dependencies(move_jSpace ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_jSpace ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(goal_publisher src/goal_publisher.cpp)
add_dependencies(goal_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(goal_publisher ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(tf_listener src/tf_listener.cpp)
add_dependencies(tf_listener ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf_listener ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(move_to_points src/move_to_points.cpp)
add_dependencies(move_to_points ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_to_points ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(add_body src/add_body.cpp)
add_dependencies(add_body ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(add_body ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(move_cSpace src/move_cSpace.cpp)
add_dependencies(move_cSpace ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_cSpace ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(move_iter_plan src/move_iter_plan.cpp)
add_dependencies(move_iter_plan ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_iter_plan ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(move_person src/move_person.cpp)
add_dependencies(move_person ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_person ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(move_cartesian src/move_cartesian.cpp)
add_dependencies(move_cartesian ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_cartesian ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(move_surfaces src/move_surfaces.cpp)
add_dependencies(move_surfaces ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_surfaces ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(trajectory_pub src/trajectory_pub.cpp)
add_dependencies(trajectory_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_pub ${catkin_LIBRARIES} ${Boost_LIBRARIES})





add_executable(into_ws src/into_ws.cpp)
add_dependencies(into_ws ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(into_ws ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(obj_tf src/obj_tf.cpp)
add_dependencies(obj_tf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(obj_tf ${catkin_LIBRARIES} ${Boost_LIBRARIES})



add_executable(matrix_storer src/matrix_storer.cpp)
add_dependencies(matrix_storer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(matrix_storer ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(matrix_storer_5 src/matrix_storer_5.cpp)
add_dependencies(matrix_storer_5 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(matrix_storer_5 ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(matrix_storer_18 src/matrix_storer_18.cpp)
add_dependencies(matrix_storer_18 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(matrix_storer_18 ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(matrix_storer_28 src/matrix_storer_28.cpp)
add_dependencies(matrix_storer_28 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(matrix_storer_28 ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(matrix_storer_slow28 src/matrix_storer_slow28.cpp)
add_dependencies(matrix_storer_slow28 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(matrix_storer_slow28 ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(matrix_storer_28fast src/matrix_storer_28fast.cpp)
add_dependencies(matrix_storer_28fast ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(matrix_storer_28fast ${catkin_LIBRARIES} ${Boost_LIBRARIES})


add_executable(plan_reader src/plan_reader.cpp)
add_dependencies(plan_reader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(plan_reader ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(matrix_reader src/matrix_reader.cpp)
add_dependencies(matrix_reader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(matrix_reader ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(matrix_reader_5 src/matrix_reader_5.cpp)
add_dependencies(matrix_reader_5 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(matrix_reader_5 ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(matrix_reader_18 src/matrix_reader_18.cpp)
add_dependencies(matrix_reader_18 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(matrix_reader_18 ${catkin_LIBRARIES} ${Boost_LIBRARIES})



add_executable(velocity_scaling src/velocity_scaling.cpp)
add_dependencies(velocity_scaling ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(velocity_scaling ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(des_frame_pub src/des_frame_pub.cpp)
add_dependencies(des_frame_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(des_frame_pub ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(des_point_pub src/des_point_pub.cpp)
add_dependencies(des_point_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(des_point_pub ${catkin_LIBRARIES} ${Boost_LIBRARIES})


add_executable(sphere_rosBridge src/sphere_rosBridge.cpp)
add_dependencies(sphere_rosBridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sphere_rosBridge ${catkin_LIBRARIES} ${Boost_LIBRARIES})



#add_executable(ur_kin src/ur_kin.cpp)
#add_dependencies(ur_kin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(ur_kin ${catkin_LIBRARIES} ${Boost_LIBRARIES})


## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDEmy_plan2.trajectory_.joint_trajectory
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_motion_planning.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

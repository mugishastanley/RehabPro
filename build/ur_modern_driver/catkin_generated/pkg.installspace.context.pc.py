# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "actionlib;control_msgs;controller_manager;geometry_msgs;hardware_interface;industrial_msgs;roscpp;sensor_msgs;trajectory_msgs;ur_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lur_hardware_interface".split(';') if "-lur_hardware_interface" != "" else []
PROJECT_NAME = "ur_modern_driver"
PROJECT_SPACE_DIR = "/home/adminuser/ws_moveit3/install"
PROJECT_VERSION = "0.1.0"

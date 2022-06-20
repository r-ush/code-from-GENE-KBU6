# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;std_msgs;message_runtime".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lelmo_ethercat;-lelmo_cest_tt;-ltestmain".split(';') if "-lelmo_ethercat;-lelmo_cest_tt;-ltestmain" != "" else []
PROJECT_NAME = "elmo_ethercat"
PROJECT_SPACE_DIR = "/home/gene/catkin_ws/install"
PROJECT_VERSION = "0.0.0"

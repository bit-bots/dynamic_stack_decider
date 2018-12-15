cmake_minimum_required(VERSION 2.8.3)
project(dynamic_stack_decider_visualization)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

install(FILES plugin_dsd_viz.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/dsd_visualization
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

cmake_minimum_required(VERSION 2.8.3)
project(dynamic_stack_decider)

find_package(catkin REQUIRED COMPONENTS rospy bitbots_docs)

catkin_python_setup()
enable_bitbots_docs()

catkin_package(
)


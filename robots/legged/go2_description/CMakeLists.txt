cmake_minimum_required(VERSION 3.10)
project(go2_description)
find_package(catkin REQUIRED)
catkin_package()
install(DIRECTORY urdf config meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

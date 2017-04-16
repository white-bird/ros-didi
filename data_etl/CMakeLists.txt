cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#AUX_SOURCE_DIRECTORY(src/process DIR_SRCS)
#AUX_SOURCE_DIRECTORY(src/xmlgen XML_DIR_SRCS)


#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
#rosbuild_add_executable(velListener src/Vellistener.cpp)
#rosbuild_add_executable(imgListener src/Imagelistener.cpp)
#rosbuild_add_executable(testCPP src/testc.cpp src/add.c src/add.h)
rosbuild_add_executable(velListener src/vel/Vellistener.cpp)
rosbuild_add_executable(imgListener src/ima/Imagelistener.cpp)
#rosbuild_add_executable(process ${DIR_SRCS} include/yingzz/pos.h)
#rosbuild_add_executable(xmlgen ${XML_DIR_SRCS} include/yingzz/pos.h)


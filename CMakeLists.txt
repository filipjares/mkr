cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(listener src/listener.cpp src/clipper.cpp src/poly2vd.cpp src/Color.cpp src/VroniUtils.cpp)
rosbuild_add_executable(odometry src/odometry.cpp)
rosbuild_add_executable(poly2vd src/poly2vd.cpp src/Color.cpp src/VroniUtils.cpp)
rosbuild_add_compile_flags(poly2vd -DPOLY2VD_STANDALONE) # poly2vd's own main()


include_directories(VRONI_6.0/src)
target_link_libraries(listener ${PROJECT_SOURCE_DIR}/VRONI_6.0/build/src/libvroni.so)
target_link_libraries(poly2vd ${PROJECT_SOURCE_DIR}/VRONI_6.0/build/src/libvroni.so)
